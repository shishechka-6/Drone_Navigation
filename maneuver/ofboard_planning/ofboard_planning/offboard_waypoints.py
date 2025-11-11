import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleCommand, TrajectorySetpoint, OffboardControlMode
import time

class OffboardControl(Node):
    def __init__(self):
        super().__init__('offboard_control')

        # Publishers
        self.offboard_control_mode_pub = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            10)
        self.trajectory_setpoint_pub = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            10)
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            10)

        # Таймер публикации (50 мс = 20 Гц)
        self.timer = self.create_timer(0.05, self.timer_callback)
        self.start_time = time.time()
        self.stage = 0

        # Задаём список точек (ENU: вперед, вбок, вниз)
        self.waypoints = [
            (0.0, 0.0, -2.0),   # Взлёт на высоту 2 м
            (0.0, 5.0, -2.0),   # Вперёд 2 м
            (-20.0, 5.0, -2.0),   # Вправо 2 м
            (0.0, 3.0, -2.0),   # Назад к исходной линии
            (0.0, 0.0, -2.0),   # Вернуться к точке взлёта
        ]
        self.current_wp = 0

        # Первое действие — включить offboard и армировать
        self.arm()
        self.set_mode_offboard()

    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.get_logger().info("Arm command sent")

    def disarm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
        self.get_logger().info("Disarm command sent")

    def set_mode_offboard(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
        self.get_logger().info("Offboard mode command sent")

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_pub.publish(msg)

    def publish_trajectory_setpoint(self, x, y, z):
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = 0.0  # фиксируем курс
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_pub.publish(msg)

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = param1
        msg.param2 = param2
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_pub.publish(msg)

    def timer_callback(self):
        self.publish_offboard_control_mode()

        if self.current_wp < len(self.waypoints):
            x, y, z = self.waypoints[self.current_wp]
            self.publish_trajectory_setpoint(x, y, z)
            self.get_logger().info(f"Flying to waypoint {self.current_wp}: ({x}, {y}, {z})")

            # Простая задержка ~5 секунд на точку
            if time.time() - self.start_time > 5.0:
                self.start_time = time.time()
                self.current_wp += 1
        else:
            self.disarm()
            self.get_logger().info("Mission completed, disarming...")
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = OffboardControl()
    rclpy.spin(node)

if __name__ == '__main__':
    main()