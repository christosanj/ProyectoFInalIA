#
# MOBILE ROBOTS - FI-UNAM, 2026-1
# GENERALIZED VORONOI DIAGRAMS
#
# Instructions:
# Implement the Brushfire algorithm to get the set of cells
# equidistant to the two nearest obstacles, given an occupancy grid map. 
#
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap
import numpy
import queue
import sys
import math
    
FULL_NAME = "CHRISTOPHER SAN JUAN FLORES"
    
class GVDNode(Node):
    
    def brushfire(self,grid_map):
        print("Executing brushfire algorithm...")
        distances = numpy.full(grid_map.shape, -1.0)
        [height, width] = grid_map.shape
        offsets_4 = [[1,0],[0,1],[-1,0],[0,-1]]
        offsets_8 = [[1,1],[-1,1],[-1,-1],[1,-1]]
    
        Q = queue.Queue()
        for i in range(height):
            for j in range(width):
                if grid_map[i,j] != 0:
                    distances[i,j] = 0
                    if i > 0 and i < height-1 and j > 0 and j < width-1:
                        Q.put([i,j])
    
        while not Q.empty():
            sys.stdout.write("\rRemaining cells: %d            " % (Q.qsize()))
            sys.stdout.flush()
            [i,j] = Q.get_nowait()
            d = distances[i,j]
            for k1, k2 in offsets_4:
                if (i+k1) < 0 or (i+k1) >= height or (j+k2)<0 or (j+k2)>=width:
                    continue
                if distances[i+k1, j+k2] == -1:
                    Q.put([i+k1, j+k2])
                    distances[i+k1, j+k2] = d + 1
                else:
                    distances[i+k1, j+k2] = min(distances[i+k1, j+k2], d+1)
            for k1, k2 in offsets_8:
                if (i+k1) < 0 or (i+k1) >= height or (j+k2)<0 or (j+k2)>=width:
                    continue
                if distances[i+k1, j+k2] == -1:
                    Q.put([i+k1, j+k2])
                    distances[i+k1, j+k2] = d + 2
                    #distances[i+k1, j+k2] = d + math.sqrt(2.0)
                else:
                    distances[i+k1, j+k2] = min(distances[i+k1, j+k2], d+ 2)
                    #distances[i+k1, j+k2] = min(distances[i+k1, j+k2], d+ math.sqrt(2.0))
        return distances

    def find_maxima(self, distances):
        [height, width] = distances.shape
        maxima = numpy.full(distances.shape, 0)
        offsets = [[1,0], [1,1], [0,1], [-1,1], [-1,0], [-1,-1], [0,-1], [1,-1]]
        for i in range(height):
            for j in range(width):
                if distances[i,j] == 0:
                    continue
                is_maximum = False
                for k1, k2 in offsets:
                    if (i+k1) < 0 or (i+k1) >= height or (j+k2)<0 or (j+k2)>=width:
                        continue
                    if (i-k1) < 0 or (i-k1) >= height or (j-k2)<0 or (j-k2)>=width:
                        continue
                    is_maximum |= (distances[i+k1, j+k2] <= distances[i,j] and distances[i-k1, j-k2] <  distances[i,j])
                if is_maximum:
                    maxima[i,j] = 100
        return maxima

    def callback_timer(self):
        self.voronoi_map.header.frame_id = "map"
        self.voronoi_map.header.stamp = self.get_clock().now().to_msg()
        self.pub_map.publish(self.voronoi_map)
        return
    
    
    def __init__(self):
        print("INITIALIZING GVD NODE - ", FULL_NAME)
        super().__init__("gvd_node")
        self.clt_inflated_map = self.create_client(GetMap, '/get_inflated_map')
        print("Waiting for inflated map service...")
        while not self.clt_inflated_map.wait_for_service(timeout_sec=1.0):
            print('Waiting for inflated map service...')
        print("Inflated map service is now available...")
        print("Trying to get first inflated map...")
        future = self.clt_inflated_map.call_async(GetMap.Request())
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        self.inflated_map = response.map
        print("Got inflated map.")
        map_info = self.inflated_map.info
        width, height, res = map_info.width, map_info.height, map_info.resolution
        grid_map = numpy.reshape(numpy.asarray(self.inflated_map.data, dtype='int'), (height, width))
        print("Inflated map with shape: " + str(grid_map.shape))
        distances = self.brushfire(grid_map)
        voronoi_data = self.find_maxima(distances)
        print("Executed brushfire succesfully")

        voronoi_data = numpy.ravel(numpy.reshape(voronoi_data, (width*height, 1)))
        print("Distances shape " + str(voronoi_data.shape))
        self.voronoi_map  = OccupancyGrid(info=map_info, data=voronoi_data)
        self.pub_map   = self.create_publisher(OccupancyGrid, "/voronoi", 1)
        self.timer = self.create_timer(1.0, self.callback_timer)


def main(args=None):
    rclpy.init(args=args)
    gvd_node = GVDNode()
    rclpy.spin(gvd_node)
    gvd_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
