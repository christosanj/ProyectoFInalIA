#
# MOBILE ROBOTS - FI-UNAM, 2026-1
# SPLIT AND MERGE ALGORITHM
#
# Instructions:
# Complete the code to implement the split and merge algorithm to detect line segments. 
# Test different parameteres and compare the results.
# 

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import ColorRGBA
from sensor_msgs.msg   import LaserScan
from geometry_msgs.msg import Twist, Point
from visualization_msgs.msg import Marker
from builtin_interfaces.msg import Duration
import numpy
import math

FULL_NAME = "CHRISTOPHER SAN JUAN FLORES"

class SplitAndMergeNode(Node):
    def adjust_line_by_LSE(self, points):
        [xm,ym] = numpy.mean(points, 0)
        n,d = 0,0
        for x,y in points:
            n += (xm - x)*(ym - y)
            d += (ym - y)**2 - (xm - x)**2
        theta = 0.5*math.atan2(-2*n , d)
        rho   = xm*math.cos(theta) + ym*math.sin(theta)
        length= numpy.linalg.norm(points[0] - points[-1])
        return rho, theta, xm, ym, length

    def find_farthest_point(self, points, rho, theta):
        distances = [abs(points[i][0]*math.cos(theta) + points[i][1]*math.sin(theta) - rho) for i in range(len(points))]
        idx = numpy.argmax(distances)
        return idx, distances[idx]

    def split(self, points, threshold, min_points):
        lines = []
        #
        # TODO:
        # Implement the 'split' part of the split and merge algorithm for finding lines.
        # Implement the recursive method of the algorithm. 
        #
        if len(points) < min_points:
            return lines
        rho, theta, xm, ym, length = self.adjust_line_by_LSE(points)
        idx, dist = self.find_farthest_point(points, rho, theta)
        if dist < threshold:
            return [[rho, theta, xm, ym, length]]
        lines1 = self.split(points[0:idx], threshold, min_points)
        lines2 = self.split(points[idx+1:len(points)], threshold, min_points)
        lines = lines1 + lines2
        return lines

    def merge(self, lines, rho_tol, theta_tol):
        new_lines = []
        #
        # TODO:
        # Implement the 'merge' part of the split and merge algorithm.
        # Two segments are merged into one if rho and theta differences
        # are both smaller than a tolerance.
        #
        if len(lines) < 2:
            return lines
        for i in range(1, len(lines)):
            rho1, theta1, xm1, ym1, length1 = lines[i]
            rho2, theta2, xm2, ym2, length2 = lines[i-1]
            e_rho = abs((rho1 - rho2)/min(rho1, rho2))
            e_theta = abs(theta1 - theta2)
            if e_rho < rho_tol and e_theta < theta_tol:
                new_lines.append([(rho1+rho2)/2, (theta1+theta2)/2, (xm1+xm2)/2, (ym1+ym2)/2, length1+length2])
            else:
                new_lines.append([rho1, theta1, xm1, ym1, length1])
                new_lines.append([rho2, theta2, xm2, ym2, length2])

        return new_lines

    def split_and_merge(self, points, threshold, min_points, rho_tol, theta_tol):
        lines = self.split(points, threshold, min_points)
        lines = self.merge(lines, rho_tol, theta_tol)
        return lines

    def get_line_markers(self, lines):
        marker = Marker()
        marker.header.frame_id = "/base_link";
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "segmented_lines";
        marker.id = 0;
        marker.type = Marker.LINE_LIST;
        marker.action = Marker.ADD;
        marker.scale.x = 0.1;
        marker.pose.orientation.w = 1.0
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
        marker.frame_locked = True
        marker.lifetime = Duration(sec=5, nanosec=0)
        for [rho, theta, xm, ym, length] in lines:
            a  = math.cos(theta)
            b  = math.sin(theta)
            p1 = Point()
            p2 = Point()
            p1.x, p1.y, p1.z = xm + length/2*(-b), ym + length/2*(a), 0.5
            p2.x, p2.y, p2.z = xm - length/2*(-b), ym - length/2*(a), 0.5
            marker.points.append(p1)
            marker.points.append(p2)
        return marker

    def callback_scan(self, msg):
        points = []
        for i in range(len(msg.ranges)):
            if not (math.isnan(msg.ranges[i]) or msg.ranges[i] >= msg.range_max):
                r,theta = msg.ranges[i], i*msg.angle_increment + msg.angle_min
                points.append([r*math.cos(theta), r*math.sin(theta)])
        points = numpy.asarray(points)
        lines = self.split_and_merge(points, self.distance_threshold, self.min_points_counting, self.rho_tolerance, self.theta_tolerance)
        self.pub_line_markers.publish(self.get_line_markers(lines))
        return

    def __init__(self):
        print("INITIALIZING SPLIT AND MERGE NODE - ", FULL_NAME)
        super().__init__("split_and_merge_node")
        
        #
        # TODO:
        # Modify the following parameters and compare the results:
        #
        self.declare_parameter("dist", 1.0)     #Distance threshold to consider a point as part of a candidate line. 
        self.declare_parameter("points", 2)     #Minimum number of points a line should contain.
        self.declare_parameter("rho", 1.0)     #RHO and THETA error tolerance to consider two lines as one.
        self.declare_parameter("theta", 1.0)
        #
        #
        #
        self.distance_threshold   = self.get_parameter('dist').get_parameter_value().double_value
        self.min_points_counting  = self.get_parameter('points').get_parameter_value().integer_value
        self.rho_tolerance        = self.get_parameter('rho').get_parameter_value().double_value
        self.theta_tolerance      = self.get_parameter('theta').get_parameter_value().double_value
        print("Trying to find lines with parameters:")
        print("Distance threshold: " + str(self.distance_threshold))
        print("Min points per line: " + str(self.min_points_counting))
        print("Rho tolerance: " + str(self.rho_tolerance))
        print("Theta tolerance: " + str(self.theta_tolerance))
        
        self.sub_scan = self.create_subscription(LaserScan, '/scan', self.callback_scan, 1)
        self.pub_line_markers = self.create_publisher(Marker, "/navigation/segmented_lines_marker", 1)


def main(args=None):
    rclpy.init(args=args)
    split_and_merge_node = SplitAndMergeNode()
    rclpy.spin(split_and_merge_node)
    split_and_merge_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
