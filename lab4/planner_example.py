import rclpy
from rclpy.node import Node


from autoware_auto_msgs.msg import BoundingBoxArray
from autoware_auto_msgs.msg import RawControlCommand
from nav_msgs.msg import Odometry


class Driver(Node):

    def __init__(self):
        super().__init__('Driver')
        self.pub = self.create_publisher(RawControlCommand,
                '/vehicle_cmd', 5)
        self.create_subscription(BoundingBoxArray, '/lidar_bounding_boxes',
                self.bounding_boxes_callback, 5)
        #self.create_subscription(Odometry, '/lgsvl_odom', self.odom_callback, 5)

        self.tmr = self.create_timer(0.25, self.controller_callback)

        self.dis = 7.78

        self.velocity = 0.0

        self.old_dis = 0

    def controller_callback(self):
        msg = RawControlCommand()
        if self.dis < 6.0:
            msg.brake = 750
            self.pub.publish(msg)
            print('brake')
        elif self.old_dis < 10.0 and self.old_dis > self.dis:
            msg.brake = 400
            self.pub.publish(msg)
            print('approaching')                 
        elif self.dis >= 6.0 and self.old_dis < self.dis:
            msg.throttle = 80
            self.pub.publish(msg)
            print('speed up with speed %d' % msg.throttle)
        self.old_dis = self.dis

    def bounding_boxes_callback(self, data):
        #print('There are %d bounding boxes.' % len(data.boxes))
        for box in data.boxes:
            #print('central of box is at %f %f %f.' % (box.centroid.x, box.centroid.y, box.centroid.z))
            if box.centroid.x >= 0 and box.centroid.y <= 0 and box.centroid.y >= -1:
                print('front car is at %f %f %f' % (box.centroid.x, box.centroid.y, box.centroid.z))
                self.dis = box.centroid.x 
                if self.dis < 7.0:
                    self.controller_callback()

    def odom_callback(self, data):
        self.velocity = data.twist.twist.linear.x
        print('velocity is %f' % self.velocity)


rclpy.init()
node = Driver()
try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass

node.destroy_node()
rclpy.shutdown()


