#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math

class MovementsNode(Node):
    def __init__(self):
        super().__init__('movements_node')
        self.publisher_ = self.create_publisher(Twist, '/robot/cmd_vel', 10)
        self.time = 0.0
        self.state = 0  # 0 - рисует круг, 1 - поворот, 2 - движение вперед/назад
        self.timer_period = 0.1  # секунды
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def timer_callback(self):
        twist_msg = Twist()
        
        if self.state == 0:
            # Рисование круга: линейная скорость вперед и угловая скорость для вращения
            twist_msg.linear.x = 0.5  # линейная скорость
            twist_msg.angular.z = 0.5  # угловая скорость для круга
            # Переход к следующему состоянию, после 5 секунд
            if self.time > 5:
                self.state = 1
                self.time = 0.0

        elif self.state == 1:
            # Поворот вокруг своей оси
            twist_msg.linear.x = 0.0  # нет движения вперед
            twist_msg.angular.z = 1.0  # угловая скорость для поворота
            # Переход к следующему состоянию, после 3 секунд
            if self.time > 3:
                self.state = 2
                self.time = 0.0

        elif self.state == 2:
            # Короткие движения вперед/назад
            if int(self.time / 1) % 2 == 0:
                twist_msg.linear.x = 0.2  # движение вперед
            else:
                twist_msg.linear.x = -0.2  # движение назад
            twist_msg.angular.z = 0.0  # нет угловой скорости
            # Переход к следующему состоянию, после 10 секунд
            if self.time > 10:
                self.state = 0
                self.time = 0.0
        
        self.publisher_.publish(twist_msg)
        self.time += self.timer_period

def main(args=None):
    rclpy.init(args=args)
    node = MovementsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
