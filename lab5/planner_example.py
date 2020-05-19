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

        self.tmr = self.create_timer(1.0, self.controller_callback)

    def controller_callback(self):
        msg = RawControlCommand()
        msg.throttle = 100
        self.pub.publish(msg)

    def bounding_boxes_callback(self, data):
        print('There are %d bounding boxes.' % len(data.boxes))
        for box in data.boxes:
            #print('central of box is at %f %f %f.' % 
            #        (box.centroid.x, box.centroid.y, box.centroid.z))
        #TODO

    def odom_callback(self, data):
        position = data.pose.pose.position
        print('Current pos: %f, %f, %f' % (position.x, position.y, position.z))
        #TODO

checkPoints = []
with open('waypoints.csv') as f:
    line = f.readline()
    while line:
        x, y = [float(v) for v in line.split(',')]
        checkPoints.append(CheckPoint(x, y))
        line = f.readline()

currentCheckPoint = 0

rclpy.init()
node = Driver()
try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass

node.destroy_node()
rclpy.shutdown()


