# Copyright 1996-2024 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
my_driver controller - Real RC Car Interface (micro-ROS compatible) for Webots.

Subscriptions:
  - /servo        (std_msgs/msg/Int8)   : Steering input [-6, 6] (Left to Right, 0 = center)
  - /motor_cmd    (std_msgs/msg/String) : Motor command ('s <rpm>' [-500, 500], 'x' = stop)
  - /motor_speed  (std_msgs/msg/Int16)  : Direct numeric speed input [-500, 500]
  - /LEDs         (std_msgs/msg/Int8)   : Lighting control matching ESP32 firmware:
                                            0: Turn signals OFF
                                            1: Left turn signal ON
                                            2: Right turn signal ON
                                            3: Hazard blinkers ON
                                            4: Front headlights ON
                                            5: Front headlights OFF
                                            6: Brake lights ON
                                            7: Brake lights OFF
  - /brake_light  (std_msgs/msg/Bool)   : Dedicated brake light toggle (True = ON, False = OFF)

Publications:
  - /camera_raw   (sensor_msgs/msg/Image) : Roof camera live feed (640x540 bgr8)
  - /motor_status (std_msgs/msg/Int8)     : Motor state (0 = Stopped, 1 = Running)
"""

import os
import sys

# Auto-bootstrap ROS 2 environment if Webots was launched from desktop without sourcing ROS 2
try:
    import rclpy
except ModuleNotFoundError:
    if "ROS_BOOTSTRAPPED" not in os.environ:
        import subprocess
        os.environ["ROS_BOOTSTRAPPED"] = "1"
        script_path = os.path.abspath(__file__)
        args_str = " ".join(f'"{a}"' for a in sys.argv[1:])
        cmd = f'source ~/.bashrc 2>/dev/null; source /opt/ros/jazzy/setup.bash 2>/dev/null; exec python3 -u "{script_path}" {args_str}'
        sys.exit(subprocess.call(["/bin/bash", "-c", cmd]))
    else:
        print("[Error] rclpy could not be found. Please ensure ROS 2 Jazzy is installed in /opt/ros/jazzy.", file=sys.stderr)
        raise

import numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Int8, Int16, String
from vehicle import Driver

# Patch Webots R2025a upstream binding bug if wb_driver_set_indicator is invoked instead of wbu_driver_set_indicator
try:
    if hasattr(Driver, 'setIndicator'):
        _orig_set_indicator = Driver.setIndicator

        def _safe_set_indicator(self, indicator: int):
            if hasattr(Driver, 'api') and hasattr(Driver.api, 'wbu_driver_set_indicator'):
                Driver.api.wbu_driver_set_indicator(indicator)
            else:
                _orig_set_indicator(self, indicator)

        Driver.setIndicator = _safe_set_indicator
except Exception:
    pass

try:
    from cv_bridge import CvBridge
    HAS_CV_BRIDGE = True
except ImportError:
    HAS_CV_BRIDGE = False


class WebotsRCControllerNode(Node):
    """
    ROS 2 Controller node replicating the physical ESP32 RC car actuator interface in Webots.
    """
    # Actuator limits & scaling factors
    MAX_STEERING_ANGLE = 0.5   # radians (~28.6 degrees physical wheel lock)
    MAX_KMH = 25.0             # km/h corresponding to 500 RPM
    MAX_RPM = 500.0            # Max RPM from real robot controller

    def __init__(self, driver):
        super().__init__('webots_rc_controller')

        self.driver = driver
        self.basic_time_step = int(self.driver.getBasicTimeStep())
        self.bridge = CvBridge() if HAS_CV_BRIDGE else None

        # QoS Profile matching the Jetson's state_node / micro-ROS best-effort communication
        qos_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # 1. Hardware Initialization: Connect and Enable Roof Camera
        self.camera = self.driver.getDevice('camera')
        if self.camera:
            self.camera.enable(self.basic_time_step)
            self.cam_width = self.camera.getWidth()
            self.cam_height = self.camera.getHeight()
            self.cam_fov = self.camera.getFov()
            self.get_logger().info(
                f"[Camera] Connected '{self.camera.getName()}': {self.cam_width}x{self.cam_height}, "
                f"FOV={self.cam_fov:.2f} rad, refresh={self.basic_time_step} ms"
            )
        else:
            self.cam_width, self.cam_height, self.cam_fov = 640, 540, 1.2
            self.get_logger().warn("[Camera] Warning: Device 'camera' not found on vehicle!")

        # 2. ROS 2 Publishers
        self.camera_pub = self.create_publisher(Image, '/camera_raw', 10)
        self.status_pub = self.create_publisher(Int8, '/motor_status', qos_best_effort)
        self.get_logger().info("Publishing camera on: /camera_raw")
        self.get_logger().info("Publishing motor status on: /motor_status")

        # 3. ROS 2 Subscribers (Matching Real Robot micro-ROS interface)
        self.servo_sub = self.create_subscription(
            Int8,
            '/servo',
            self.servo_callback,
            qos_best_effort
        )
        self.motor_cmd_sub = self.create_subscription(
            String,
            '/motor_cmd',
            self.motor_cmd_callback,
            qos_best_effort
        )
        self.motor_speed_sub = self.create_subscription(
            Int16,
            '/motor_speed',
            self.motor_speed_callback,
            qos_best_effort
        )
        # Lighting & Turn Signals & Brake Subscribers
        self.leds_sub = self.create_subscription(
            Int8,
            '/LEDs',
            self.leds_callback,
            qos_best_effort
        )
        self.brake_light_sub = self.create_subscription(
            Bool,
            '/brake_light',
            self.brake_light_callback,
            qos_best_effort
        )

        self.get_logger().info("Subscribed to: /servo (Int8 [-6..6])")
        self.get_logger().info("Subscribed to: /motor_cmd (String 's <rpm>', 'x')")
        self.get_logger().info("Subscribed to: /motor_speed (Int16 [-500..500])")
        self.get_logger().info("Subscribed to: /LEDs (Int8 [0..7: signals, headlights, brakes])")
        self.get_logger().info("Subscribed to: /brake_light (Bool [True=ON, False=OFF])")

        # 4. State Variables
        self.current_servo_val = 0
        self.current_rpm = 0.0
        self.current_kmh = 0.0
        self.is_running = False
        self.frame_count = 0
        self.last_status_time = 0.0

        # Initial vehicle state (stationary, straight, headlights on)
        self.driver.setCruisingSpeed(0.0)
        self.driver.setSteeringAngle(0.0)
        self.driver.setDippedBeams(True)
        self.driver.setHazardFlashers(False)
        self.driver.setIndicator(Driver.INDICATOR_OFF)
        self.driver.setBrakeIntensity(0.0)

    def servo_callback(self, msg: Int8):
        """
        Handles /servo input: [-6, 6] (Left to Right, 0 = center).
        Maps linearly to Webots steering knuckle angle [-0.5, 0.5] rad.
        """
        raw_val = int(msg.data)
        constrained_val = max(-6, min(6, raw_val))
        self.current_servo_val = constrained_val

        # Webots: positive = turn right, negative = turn left
        steering_angle = (constrained_val / 6.0) * self.MAX_STEERING_ANGLE
        self.driver.setSteeringAngle(steering_angle)

        self.get_logger().info(f"[Servo] Input: {raw_val:2d} -> Steering Angle: {steering_angle:+6.3f} rad ({constrained_val * 10:+} deg)")

    def motor_cmd_callback(self, msg: String):
        """
        Handles /motor_cmd input string:
          - 's <rpm>' : Continuous speed in [-500, 500] (negative = backward, positive = forward)
          - 'x'       : Emergency stop / brake
          - 'r'       : Reset motor
        """
        cmd_text = msg.data.strip()
        if not cmd_text:
            return

        cmd = cmd_text[0]

        if cmd == 's':
            parts = cmd_text.split()
            if len(parts) >= 2:
                try:
                    rpm = float(parts[1])
                    self.set_motor_rpm(rpm)
                except ValueError:
                    self.get_logger().warn(f"[Motor] Invalid speed format: '{cmd_text}'")
            else:
                self.stop_motor()

        elif cmd == 'x' or cmd == 'r':
            self.stop_motor()
            # Brief visual brake light flash when stopping
            self.driver.setBrakeIntensity(1.0)
            self.get_logger().info(f"[Motor] Stop command received ('{cmd}') -> Brakes applied")

        elif cmd in ('c', 'm'):
            parts = cmd_text.split()
            rpm = float(parts[2]) if len(parts) >= 3 else 60.0
            self.set_motor_rpm(rpm)
            self.get_logger().info(f"[Motor] Move command '{cmd_text}' executed at {rpm:.1f} RPM")

    def motor_speed_callback(self, msg: Int16):
        """
        Handles direct numeric speed input on /motor_speed in [-500, 500].
        """
        self.set_motor_rpm(float(msg.data))

    def leds_callback(self, msg: Int8):
        """
        Handles /LEDs input matching ESP32 firmware & Jetson state_node:
          0: Both indicators OFF
          1: Left turn signal ON
          2: Right turn signal ON
          3: Both ON (Hazard blinkers)
          4: Front headlights (dipped beams) ON
          5: Front headlights OFF
          6: Brake lights ON
          7: Brake lights OFF
        """
        val = int(msg.data)
        if val == 0:
            self.driver.setHazardFlashers(False)
            self.driver.setIndicator(Driver.INDICATOR_OFF)
            self.get_logger().info("[LEDs] Turn signals OFF")
        elif val == 1:
            self.driver.setHazardFlashers(False)
            self.driver.setIndicator(Driver.INDICATOR_LEFT)
            self.get_logger().info("[LEDs] Left turn signal blinking")
        elif val == 2:
            self.driver.setHazardFlashers(False)
            self.driver.setIndicator(Driver.INDICATOR_RIGHT)
            self.get_logger().info("[LEDs] Right turn signal blinking")
        elif val == 3:
            self.driver.setIndicator(Driver.INDICATOR_OFF)
            self.driver.setHazardFlashers(True)
            self.get_logger().info("[LEDs] Hazard emergency flashers ON")
        elif val == 4:
            self.driver.setDippedBeams(True)
            self.get_logger().info("[LEDs] Front headlights ON")
        elif val == 5:
            self.driver.setDippedBeams(False)
            self.get_logger().info("[LEDs] Front headlights OFF")
        elif val == 6:
            self.driver.setBrakeIntensity(1.0)
            self.get_logger().info("[LEDs] Brake lights ON (Red)")
        elif val == 7:
            self.driver.setBrakeIntensity(0.0)
            self.get_logger().info("[LEDs] Brake lights OFF")
        else:
            self.get_logger().warn(f"[LEDs] Unknown command code: {val}")

    def brake_light_callback(self, msg: Bool):
        """
        Dedicated toggle for brake lights on /brake_light (True = ON, False = OFF).
        """
        if msg.data:
            self.driver.setBrakeIntensity(1.0)
            self.get_logger().info("[Brake] Brake lights ON (Red)")
        else:
            self.driver.setBrakeIntensity(0.0)
            self.get_logger().info("[Brake] Brake lights OFF")

    def set_motor_rpm(self, rpm: float):
        """
        Scales target RPM [-500, 500] to Webots cruising speed [-MAX_KMH, MAX_KMH].
        Positive = Forward, Negative = Backward, 0 = Stop.
        """
        clamped_rpm = max(-self.MAX_RPM, min(self.MAX_RPM, rpm))
        self.current_rpm = clamped_rpm

        if abs(clamped_rpm) < 1.0:
            self.stop_motor()
            return

        # Release brakes when moving
        self.driver.setBrakeIntensity(0.0)

        target_kmh = (clamped_rpm / self.MAX_RPM) * self.MAX_KMH
        self.current_kmh = target_kmh
        self.is_running = True

        self.driver.setCruisingSpeed(target_kmh)
        direction_str = "FORWARD" if target_kmh > 0 else "BACKWARD"
        self.get_logger().info(
            f"[Motor] RPM: {clamped_rpm:+6.1f} -> Speed: {target_kmh:+5.1f} km/h ({direction_str})"
        )

    def stop_motor(self):
        """Stops vehicle motion."""
        self.current_rpm = 0.0
        self.current_kmh = 0.0
        self.is_running = False
        self.driver.setCruisingSpeed(0.0)

    def publish_frame(self):
        """Fetches current camera frame and publishes to /camera_raw."""
        if not self.camera:
            return

        raw_img = self.camera.getImage()
        if not raw_img:
            return

        now = self.get_clock().now().to_msg()
        img_np = np.frombuffer(raw_img, dtype=np.uint8).reshape((self.cam_height, self.cam_width, 4))
        bgr_frame = img_np[:, :, :3]

        if self.bridge:
            msg = self.bridge.cv2_to_imgmsg(bgr_frame, encoding="bgr8")
            msg.header.stamp = now
            msg.header.frame_id = 'camera_link'
        else:
            msg = Image()
            msg.header.stamp = now
            msg.header.frame_id = 'camera_link'
            msg.height = self.cam_height
            msg.width = self.cam_width
            msg.encoding = 'bgr8'
            msg.is_bigendian = 0
            msg.step = self.cam_width * 3
            msg.data = bgr_frame.tobytes()

        self.camera_pub.publish(msg)
        self.frame_count += 1

        if self.frame_count == 1 or self.frame_count % 200 == 0:
            self.get_logger().info(f"[Camera] Published frame #{self.frame_count} on /camera_raw")

    def publish_status(self):
        """Publishes /motor_status (0 = Stopped, 1 = Running) at 5 Hz."""
        sim_time = self.driver.getTime()
        if sim_time - self.last_status_time >= 0.2:  # 5 Hz
            self.last_status_time = sim_time
            status_msg = Int8()
            status_msg.data = 1 if self.is_running else 0
            self.status_pub.publish(status_msg)


def main(args=None):
    rclpy.init(args=args)

    driver = Driver()
    node = WebotsRCControllerNode(driver)

    try:
        # Main simulation loop: step Webots simulation and process ROS 2 callbacks
        while driver.step() != -1 and rclpy.ok():
            node.publish_frame()
            node.publish_status()
            rclpy.spin_once(node, timeout_sec=0)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Shutting down WebotsRCControllerNode...")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
