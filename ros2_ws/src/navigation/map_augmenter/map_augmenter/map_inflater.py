#
# MOBILE ROBOTS - FI-UNAM, 2026-1
# MAP INFLATION 
#
# Instructions:
# Complete the code necessary to inflate the obstacles given an occupancy grid map and
# a number of cells to inflate.
#

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap
import numpy

FULL_NAME = "CHRISTOPHER SAN JUAN FLORES"

class MapInflaterNode(Node):
    def get_inflated_map(self, static_map, inflation_cells):
        print("Inflating map by " + str(inflation_cells) + " cells")
        inflated = numpy.copy(static_map)
        [height, width] = static_map.shape
        #
        # TODO:
        # Write the code necessary to inflate the obstacles in the map a radius
        # given by 'inflation_cells' (expressed in number of cells)
        # Map is given in 'static_map' as a bidimensional numpy array.
        # Consider as occupied cells all cells with an occupation value greater than 50
        #
        for i in range(len(static_map)):
            for j in range(len(static_map[0])):
                if(static_map[i,j] == 100):
                    for k1 in range(-inflation_cells, inflation_cells):
                        for k2 in range(-inflation_cells, inflation_cells):
                            r = min(height-1, max(0, i+k1))
                            c = min(width-1, max(0, j+k2))
                            inflated[r,c] = 100
        return inflated

    def callback_inflated_map(self, request, response):
        response.map = self.inflated_map
        return response

    def callback_timer(self):
        self.map_info   = self.map_static.info
        self.map_width  = self.map_info.width
        self.map_height = self.map_info.height
        self.map_res    = self.map_info.resolution
        self.map_data = numpy.reshape(numpy.asarray(self.map_static.data, dtype='int'), (self.map_height, self.map_width))
        inflation_radius  = self.get_parameter('inflation_radius').get_parameter_value().double_value
        inflated_map_data = self.get_inflated_map(self.map_data, round(inflation_radius/self.map_res))
        inflated_map_data = numpy.ravel(numpy.reshape(inflated_map_data, (self.map_width*self.map_height, 1)))
        self.inflated_map = OccupancyGrid(info=self.map_info, data=inflated_map_data)
        self.inflated_map.header.frame_id = "map"
        self.inflated_map.header.stamp = self.get_clock().now().to_msg()
        self.pub_inflated_map.publish(self.inflated_map)
        return

    def __init__(self):
        print("INITIALIZING MAP INFLATER NODE - ", FULL_NAME)
        super().__init__("map_inflater_node")
        self.clt_static_map = self.create_client(GetMap, '/map_server/map')
        print("Waiting for static map service...")
        while not self.clt_static_map.wait_for_service(timeout_sec=1.0):
            print('Waiting for static map service...')
        print("Static map service is now available...")
        print("Trying to get first static map...")
        future = self.clt_static_map.call_async(GetMap.Request())
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        self.map_static = response.map
        print("Got static map.")
        self.declare_parameter('inflation_radius', 0.05)
        self.srv_inflate_map  = self.create_service(GetMap, '/get_inflated_map', self.callback_inflated_map)
        self.pub_inflated_map = self.create_publisher(OccupancyGrid, '/inflated_map', 10)
        self.timer = self.create_timer(1.0, self.callback_timer)


def main(args=None):
    rclpy.init(args=args)
    map_inflater_node = MapInflaterNode()
    rclpy.spin(map_inflater_node)
    map_inflater_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
