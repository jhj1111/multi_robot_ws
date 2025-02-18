#!/usr/bin/env python3
# Copyright 2023 Georg Novotny
#
# Licensed under the GNU GENERAL PUBLIC LICENSE, Version 3.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.gnu.org/licenses/gpl-3.0.en.html
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, Bool, Int8, String
from geometry_msgs.msg import Twist, Pose, Vector3, PoseWithCovarianceStamped
from sensor_msgs.msg import Range, Image, Imu

STATES = {
    0: "Landed",
    1: "Flying",
    2: "Taking off",
    3: "Landing",
}

MODES = ["velocity", "position"]


class DroneObject(Node):
    def __init__(self, node_name: str = "drone_node"):
        super().__init__(node_name)
        self._state = STATES[0]
        self._mode = MODES[0]
        self._hover_distance = 0.0
        self.isFlying = False
        self.isPosctrl = False
        self.isVelMode = False

        self.logger = self.get_logger()

        # Publishers
        self.pubTakeOff = self.create_publisher(Empty, '/simple_drone/takeoff', 1024)
        self.pubLand = self.create_publisher(Empty, '/simple_drone/land', 1024)
        self.pubReset = self.create_publisher(Empty, '/simple_drone/reset', 1024)
        self.pubPosCtrl = self.create_publisher(Bool, '/simple_drone/posctrl', 1024)
        self.pubCmd = self.create_publisher(Twist, '/simple_drone/cmd_vel', 1024)
        self.pubVelMode = self.create_publisher(Bool, '/simple_drone/dronevel_mode', 1024)

        # Subscribers
        self.sub_sonar = self.create_subscription(Range, '/simple_drone/sonar', self.cb_sonar, 1024)
        self.sub_imu = self.create_subscription(Range, '/simple_drone/imu', self.cb_imu, 1024)
        self.sub_front_img = self.create_subscription(Image, '/simple_drone/front/image_raw',
                                                      self.cb_front_img, 1024)
        self.sub_bottom_img = self.create_subscription(Image, '/simple_drone/bottom/image_raw',
                                                       self.cb_bottom_img, 1024)
        self.sub_gt_pose = self.create_subscription(Pose, '/simple_drone/gt_pose', self.cb_gt_pose, 1024)
        self.sub_state = self.create_subscription(Int8, '/simple_drone/state', self.cb_state, 1024)
        self.sub_cmd_mode = self.create_subscription(String, '/simple_drone/cmd_mode', self.cb_cmd_mode, 1024)

        self._sonar = Range()
        self._imu = Imu()
        self._front_img = Image()
        self._bottom_img = Image()
        self._gt_pose = Pose()

        while self.pubTakeOff.get_subscription_count() == 0:
            self.logger.info("Waiting for drone to spawn", throttle_duration_sec=1)

    @property
    def state(self):
        return self._state

    @property
    def mode(self):
        return self._mode

    @property
    def hover_distance(self):
        return self._hover_distance

    @property
    def sonar(self):
        return self._sonar

    @property
    def imu(self):
        return self._imu

    @property
    def front_img(self):
        return self._front_img

    @property
    def bottom_img(self):
        return self._bottom_img

    @property
    def gt_pose(self):
        return self._gt_pose

    @state.setter
    def state(self, value):
        self._state = value

    @mode.setter
    def mode(self, value):
        self._mode = value

    @hover_distance.setter
    def hover_distance(self, value):
        self._hover_distance = value

    @sonar.setter
    def sonar(self, value):
        self._sonar = value

    @imu.setter
    def imu(self, value):
        self._imu = value

    @front_img.setter
    def front_img(self, value):
        self._front_img = value

    @bottom_img.setter
    def bottom_img(self, value):
        self._bottom_img = value

    @gt_pose.setter
    def gt_pose(self, value):
        self._gt_pose = value

    def takeOff(self):
        """
        Take off the drone
        :return: True if the command was sent successfully, False if drone is already flying
        """
        if self.isFlying:
            return False
        self.logger.info("Taking off")
        self.pubTakeOff.publish(Empty())
        self.isFlying = True
        return True

    def land(self):
        """
        Land the drone
        :return: True if the command was sent successfully, False if drone is not flying
        """
        if not self.isFlying:
            return False
        self.logger.info("Landing")
        self.pubLand.publish(Empty())
        self.isFlying = False
        return True

    def hover(self):
        """
        Hover the drone
        :return: True if the command was sent successfully, False if drone is not flying
        """
        if not self.isFlying:
            return False
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.linear.y = 0.0
        twist_msg.linear.z = 0.0
        twist_msg.angular.x = 0.0
        twist_msg.angular.y = 0.0
        twist_msg.angular.z = 0.0
        self.pubCmd.publish(twist_msg)
        return True

    def posCtrl(self, on):
        """
        Turn on/off position control
        :param on: True to turn on position control, False to turn off
        """
        self.isPosctrl = on
        bool_msg = Bool()
        bool_msg.data = on
        self.pubPosCtrl.publish(bool_msg)
        return True

    def velMode(self, on):
        """
        Turn on/off velocity control mode
        :param on: True to turn on velocity control mode, False to turn off
        :return: True if the command was sent successfully, False if drone is not flying
        """
        if not self.isFlying:
            return False
        self.isVelMode = on
        bool_msg = Bool()
        bool_msg.data = on
        self.pubVelMode.publish(bool_msg)
        return True

    def move(self, v_linear: Vector3 = Vector3(),
             v_angular: Vector3 = Vector3()):
        """
        Move the drone using velocity control along the linear x and z axis and rotation around
        the x, y and z axis
        :param v_linear: Linear velocity in m/s
        :param v_angular: Angular velocity in rad/s
        :return: True if the command was sent successfully, False if drone is not flying
        """
        if not self.isFlying:
            return False
        twist_msg = Twist(linear=v_linear, angular=v_angular)
        self.pubCmd.publish(twist_msg)
        return True

    def moveTo(self, x: float, y: float, z: float):
        """
        Move the drone to a specific position
        :param x: X position in m
        :param y: Y position in m
        :param z: Z position in m
        :return: True if the command was sent successfully, False if drone is not flying
        """
        if not self.isFlying:
            return False
        twist_msg = Twist()
        twist_msg.linear.x = x
        twist_msg.linear.y = y
        twist_msg.linear.z = z
        twist_msg.angular.x = 0.0
        twist_msg.angular.y = 0.0
        twist_msg.angular.z = 0.0
        self.pubCmd.publish(twist_msg)
        return True

    def pitch(self, speed):
        """
        Pitch the drone
        :param speed: Pitch speed in rad/s
        :return: True if the command was sent successfully, False if drone is not flying
        """
        if not self.isFlying:
            return False
        twist_msg = Twist()
        twist_msg.linear.x = 1.0
        twist_msg.linear.y = 1.0
        twist_msg.linear.z = 0.0
        twist_msg.angular.x = 0.0
        twist_msg.angular.y = speed
        twist_msg.angular.z = 0.0
        self.pubCmd.publish(twist_msg)
        return True

    def roll(self, speed: float):
        """
        Roll the drone
        :param speed: Roll speed in rad/s
        :return: True if the command was sent successfully, False if drone is not flying
        """
        if not self.isFlying:
            return False
        twist_msg = Twist()
        twist_msg.linear.x = 1.0
        twist_msg.linear.y = 1.0
        twist_msg.linear.z = 0.0
        twist_msg.angular.x = speed
        twist_msg.angular.y = 0.0
        twist_msg.angular.z = 0.0
        self.pubCmd.publish(twist_msg)
        return True

    def rise(self, speed: float):
        """
        Rise or fall the drone
        :param speed: Rise speed in m/s
        :return: True if the command was sent successfully, False if drone is not flying
        """
        if not self.isFlying:
            return False
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.linear.y = 0.0
        twist_msg.linear.z = speed
        twist_msg.angular.x = 0.0
        twist_msg.angular.y = 0.0
        twist_msg.angular.z = 0.0
        self.pubCmd.publish(twist_msg)
        return True

    def yaw(self, speed: float):
        """
        Rotate the drone around the z-axis
        :param speed: Rotation speed in rad/s
        :return: True if the command was sent successfully, False if drone is not flying
        """
        if not self.isFlying:
            return False
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.linear.y = 0.0
        twist_msg.linear.z = 0.0
        twist_msg.angular.x = 0.0
        twist_msg.angular.y = 0.0
        twist_msg.angular.z = speed
        self.pubCmd.publish(twist_msg)
        return True

    def cb_sonar(self, msg: Range):
        """Callback for the sonar sensor"""
        self._sonar = msg
        self._hover_distance = msg.min_range

    def cb_imu(self, msg: Imu):
        """Callback for the imu sensor"""
        self._imu = msg

    def cb_front_img(self, msg: Image):
        """Callback for the front camera"""
        self._front_img = msg

    def cb_bottom_img(self, msg: Image):
        """Callback for the rear camera"""
        self._bottom_img = msg

    def cb_gt_pose(self, msg: Pose) -> None:
        """Callback for the ground truth pose"""
        self._gt_pose = msg

    def cb_state(self, msg: Int8):
        """Callback for the drone state"""
        self._state = STATES[msg.data]
        self.logger.info("State: {}".format(self._state), throttle_duration_sec=1)

    def cb_cmd_mode(self, msg: String):
        """Callback for the command mode"""
        if msg.data in MODES:
            self._mode = msg.data
            self.logger.info("Changed command mode to: {}".format(self._mode))
        else:
            self.logger.error("Invalid command mode: {}".format(msg.data))

    def reset(self):
        self.pubReset.publish(Empty())



class DronePositionControl(DroneObject):
    def __init__(self):
        super().__init__('drone_position_control')

        self.takeOff()
        self.get_logger().info('Drone takeoff')

        # Set the m_posCtrl flag to True
        self.posCtrl(True)
        self.get_logger().info('Position control mode set to True')

        # 초기 위치 저장 변수
        self.initial_pose = None
        self.drone_initial_pose = None
        self.current_pose = None

        # Subscribers
        self.sub_initial_pose = self.create_subscription(
            PoseWithCovarianceStamped, '/initialpose', self.cb_initial_pose, 10)
        self.sub_amcl_pose = self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self.cb_amcl_pose, 10)
        self.sub_gt_pose = self.create_subscription(
            Pose, '/simple_drone/gt_pose', self.cb_gt_pose, 10)

        # Send a command to move the drone to a defined pose
        #self.move_drone_to_pose(10.0, 2.0, 5.0)  # Example pose coordinates
        #self.move_drone_to_pose(0.0, 0.0, 2.0)
        #self.move_drone_to_pose(-3.713309754, 2.067687185, 2.0)  # Example pose coordinates


    def move_drone_to_pose(self, x, y, z):
        # Override the move_drone_to_pose method if specific behavior is needed
        super().moveTo(x, y, z)
        self.get_logger().info(f'Moving drone to pose: x={x}, y={y}, z={z}')

    def move2point(self, dx: float, dy: float, dz: float):
        if not self.isFlying:
            return False
        
        rate = self.create_rate(10)  # 10Hz 루프
        target_reached = False
        
        while not target_reached:
            distance = (dx**2 + dy**2 + dz**2)**0.5

            if distance < 0.1:  # 목표 근처에 도달하면 정지
                target_reached = True
                self.hover()
                self.get_logger().info("Target reached")
                break

            # 속도 설정 (거리 기반)
            twist_msg = Twist()
            twist_msg.linear.x = min(max(dx, -1.0), 1.0)  # 속도 제한 (-1.0 ~ 1.0 m/s)
            twist_msg.linear.y = min(max(dy, -1.0), 1.0)
            #twist_msg.linear.z = min(max(dz, -1.0), 1.0)
            twist_msg.linear.z = 0.0

            self.pubCmd.publish(twist_msg)
            rate.sleep()

        return True

    def cb_initial_pose(self, msg: PoseWithCovarianceStamped):
        """초기 위치 저장"""
        self.initial_pose = msg.pose.pose
        self.get_logger().info('turtlebot3 initial pose received.')

    def cb_gt_pose(self, msg: Pose):
        """현재 Ground Truth 위치 저장"""
        if self.drone_initial_pose == None : self.drone_initial_pose = msg

        self.current_pose = msg
        #self.get_logger().info('Current ground truth pose updated.')

    def cb_amcl_pose(self, msg: PoseWithCovarianceStamped):
        """AMCL 위치를 받아서 이동 명령 실행"""
        if self.initial_pose is None:
            self.get_logger().warn("Initial pose not received yet. Ignoring AMCL command.")
            return

        amcl_pose = msg.pose.pose

        # 이동할 거리 계산 (AMCL 위치 - Initial 위치)
        #dx = amcl_pose.position.x - self.initial_pose.position.x
        #dy = amcl_pose.position.y - self.initial_pose.position.y
        #dz = amcl_pose.position.z - self.initial_pose.position.z  # 필요하면 사용

        drone_dx = self.current_pose.position.x - self.drone_initial_pose.position.x
        drone_dy = self.current_pose.position.y - self.drone_initial_pose.position.y
        turtlebot3_dx = amcl_pose.position.x - self.initial_pose.position.x
        turtlebot3_dy = amcl_pose.position.y - self.initial_pose.position.y

        drone_turtlebot3_dx = turtlebot3_dx - drone_dx
        drone_turtlebot3_dy = turtlebot3_dy - drone_dy

        # 드론 이동 실행
        self.move2point(drone_turtlebot3_dx, drone_turtlebot3_dy, 0.0)


def main(args=None):
    rclpy.init(args=args)
    drone_position_control_node = DronePositionControl()
    rclpy.spin(drone_position_control_node)
    drone_position_control_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()