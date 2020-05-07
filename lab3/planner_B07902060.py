import rclpy
from rclpy.node import Node


from autoware_auto_msgs.msg import BoundingBoxArray
from autoware_auto_msgs.msg import RawControlCommand
from nav_msgs.msg import Odometry


class Driver(Node):

    def __init__(self):
        super().__init__('Driver')
        self.pub = self.create_publisher(RawControlCommand,
                '/vehicle_cmd', 10)
        self.create_subscription(BoundingBoxArray, '/lidar_bounding_boxes',
                self.bounding_boxes_callback, 10)
        self.create_subscription(Odometry, '/lgsvl_odom',
                self.odom_callback, 10)

        self.tmr = self.create_timer(0.5, self.controller_callback)

        self.twist = 0

        self.cmd = True

        self.dis = 40.0

    def controller_callback(self):
        print('send command')
        msg = RawControlCommand()
        if self.cmd == False:
            if self.dis <= 15:
                msg.brake = 1000 * int(40.0 / self.dis)
            else:
                msg.brake = 750 * int(40.0 / self.dis)
        elif self.twist <= 11 and self.cmd == True:
            msg.throttle = 70
        self.pub.publish(msg)

    def bounding_boxes_callback(self, data):
        #print('There are %d bounding boxes.' % len(data.boxes))
        for box in data.boxes:
            if box.centroid.y <= -0.1 and box.centroid.y >= -1 and abs(box.centroid.z) <= 1 and box.centroid.x >= 0:
                if box.centroid.x <= 20:
                    self.cmd = False
                    self.dis = box.centroid.x
                    self.controller_callback()
                    print('central of box is at %f %f %f.' % (box.centroid.x, box.centroid.y, box.centroid.z))
            #print('central of box is at %f %f %f.' % (box.centroid.x, box.centroid.y, box.centroid.z))            

    def odom_callback(self, data):
        #position = data.pose.pose.position
        twist = data.twist.twist.linear
        #print('Current twi: %f, %f, %f' % (twist.x, twist.y, twist.z))
        self.twist = twist.x


rclpy.init()
node = Driver()
try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass

node.destroy_node()
rclpy.shutdown()


