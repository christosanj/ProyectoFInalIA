import rclpy
from rclpy.node import Node
import time
import math
import numpy
from nav_msgs.srv import GetPlan
from geometry_msgs.msg import Twist, PoseStamped, Point, Pose
from nav_msgs.msg import Path
from std_msgs.msg import Bool
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

NAME = "PROYECTO FINAL. SAN JUAN FLORES CHRISTOPHER"

EXPLORATION_POINTS = [
    numpy.asarray([0.40,  -1.00]),
    numpy.asarray([0.30,  -0.16]),
    numpy.asarray([2.00,  -2.51]),
    numpy.asarray([-1.23, -1.17]),
    numpy.asarray([-0.33, -4.73]),
    numpy.asarray([1.64,  -5.57]),
    numpy.asarray([1.89,  -2.84]),
    numpy.asarray([-1.90, -3.13]),
    numpy.asarray([-4.32, -3.16]),
    numpy.asarray([-2.18, -5.29]),
    numpy.asarray([2.74,  -5.03]),
    numpy.asarray([0.45,   1.70]),
    numpy.asarray([-2.89, -0.60]),
    numpy.asarray([-4.70, -7.58]),
    numpy.asarray([-2.13, -7.13]),
    numpy.asarray([3.60,  -2.31]),
    numpy.asarray([3.98,  -2.73]),
    numpy.asarray([4.13,  -4.50]),
    numpy.asarray([4.37,  -6.89]),
    numpy.asarray([3.39,  -6.92]),
    numpy.asarray([2.49,  -9.80]),
    numpy.asarray([2.58, -11.76]),
    numpy.asarray([2.30,  -7.40]),
    numpy.asarray([0.73,  -8.09]),
    numpy.asarray([-0.70, -8.09]),
    numpy.asarray([-0.44,-11.71]),
    numpy.asarray([-3.63,-11.60]),
    numpy.asarray([-4.02,-11.64]),
    numpy.asarray([1.75,  -0.16]),
    numpy.asarray([1.57,   0.89]),
    numpy.asarray([0.23,   3.29]),
    numpy.asarray([0.94,   4.63]),
    numpy.asarray([-0.46,  4.64]),
    numpy.asarray([-0.98,  0.95]),
    numpy.asarray([2.01,   0.29]),
    numpy.asarray([-2.32,  2.60]),
    numpy.asarray([-2.45,  4.29]),
    numpy.asarray([-3.71,  2.98]),
    numpy.asarray([-3.92,  1.48]),
    numpy.asarray([-3.30,  4.27]),
]

SM_INIT           = 0
SM_LOAD_GOAL      = 10
SM_REQUEST_PLAN   = 20
SM_FOLLOWING_PATH = 30
SM_POINT_REACHED  = 40
SM_EXPLORATION_DONE = 50


class Mission(Node):

    def calculate_control(self, goal_x, goal_y):
        robot_x = self.robot_pose[0]
        robot_y = self.robot_pose[1]
        robot_a = self.robot_a
        error_a = math.atan2(goal_y - robot_y, goal_x - robot_x) - robot_a
        error_a = (error_a + math.pi) % (2 * math.pi) - math.pi
        v = self.v_max * math.exp(-error_a * error_a / self.alpha)
        w = self.w_max * (2 / (1 + math.exp(-error_a / self.beta)) - 1)
        return [v, w]

    def get_robot_pose(self):
        try:
            t = self.tf_buffer.lookup_transform(
                "map", "base_link", rclpy.time.Time())
            robot_x = t.transform.translation.x
            robot_y = t.transform.translation.y
            self.robot_pose = numpy.asarray([robot_x, robot_y])
            self.robot_a = math.atan2(
                t.transform.rotation.z, t.transform.rotation.w) * 2
            return self.robot_pose, self.robot_a
        except TransformException:
            return numpy.asarray([0.0, 0.0]), 0.0

    def callback_stop(self, msg):
        if msg.data:
            self.get_logger().info("Objetos encontrados. Fin de la ruta.")
            self.state = SM_EXPLORATION_DONE

    def mission_loop_callback(self):
        self.get_robot_pose()

        if self.state == SM_INIT:
            if self.clt_plan_path.wait_for_service(timeout_sec=0.1):
                self.get_logger().info("A* disponible. Iniciando mision.")
                self.state = SM_LOAD_GOAL
            else:
                self.get_logger().info("Esperando A*...")

        elif self.state == SM_LOAD_GOAL:
            if all(self.visited_points):
                self.state = SM_EXPLORATION_DONE
                return

            # Elegir el punto no visitado más cercano al robot
            winner   = -1
            best_dist = 1000.0
            for indice in range(len(self.points)):
                if not self.visited_points[indice]:
                    candidate = self.points[indice]
                    dist = numpy.linalg.norm(self.robot_pose - candidate)
                    if dist < best_dist:
                        best_dist = dist
                        winner    = indice

            if winner != -1:
                self.point_idx      = winner
                self.current_goal_pose = self.points[self.point_idx]
                self.get_logger().info(
                    f"Siguiente punto [{self.point_idx}]: {self.current_goal_pose}")
                self.state = SM_REQUEST_PLAN
            else:
                self.state = SM_EXPLORATION_DONE

        elif self.state == SM_REQUEST_PLAN:
            request = GetPlan.Request()
            request.start.header.frame_id    = 'map'
            request.start.pose.position.x    = float(self.robot_pose[0])
            request.start.pose.position.y    = float(self.robot_pose[1])
            request.goal.header.frame_id     = 'map'
            request.goal.pose.position.x     = float(self.current_goal_pose[0])
            request.goal.pose.position.y     = float(self.current_goal_pose[1])
            self.future = self.clt_plan_path.call_async(request)
            self.future.add_done_callback(self.callback_plan_done)
            self.state = SM_FOLLOWING_PATH

        elif self.state == SM_FOLLOWING_PATH:
            if not self.current_path:
                return   # esperando que llegue el plan

            Pg  = self.current_path[self.path_idx]
            v, w = self.calculate_control(Pg[0], Pg[1])
            msg = Twist()
            msg.linear.x  = v
            msg.angular.z = w
            self.pub_cmd_vel.publish(msg)

            dist_to_waypoint = numpy.linalg.norm(self.robot_pose - Pg)
            if dist_to_waypoint < self.tol:
                if self.path_idx == len(self.current_path) - 1:
                    self.pub_cmd_vel.publish(Twist())
                    self.path_idx    = 0
                    self.current_path = []
                    self.state = SM_POINT_REACHED
                else:
                    self.path_idx += 1

        elif self.state == SM_POINT_REACHED:
            self.get_logger().info(f"Punto {self.point_idx} alcanzado.")
            self.visited_points[self.point_idx] = True
            self.pub_cmd_vel.publish(Twist())
            self.state = SM_LOAD_GOAL

        elif self.state == SM_EXPLORATION_DONE:
            self.get_logger().info("EXPLORACION TERMINADA O DETENIDA.")
            self.pub_cmd_vel.publish(Twist())
            self.mission_timer.cancel()

    def callback_plan_done(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().warn(f"Plan fallido: {e}. Pasando al siguiente punto.")
            self.state = SM_POINT_REACHED
            return

        if response.plan.poses:
            self.current_path = [
                numpy.asarray([p.pose.position.x, p.pose.position.y])
                for p in response.plan.poses
            ]
            self.path_idx = 0
        else:
            self.get_logger().warn("Plan vacio. Saltando punto.")
            self.current_path = []
            self.state = SM_POINT_REACHED

    def __init__(self):
        print("INITIALIZING -", NAME)
        super().__init__("robot_navigation_node")

        self.v_max = 0.5
        self.w_max = 0.5
        self.alpha = 0.3
        self.beta  = 0.1
        self.tol   = 0.3

        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.robot_pose        = numpy.asarray([0.0, 0.0])
        self.robot_a           = 0.0
        self.state             = SM_INIT
        self.points            = EXPLORATION_POINTS
        self.point_idx         = 0
        self.visited_points    = numpy.full(len(self.points), False)
        self.current_goal_pose = numpy.asarray([0.0, 0.0])
        self.current_path      = []
        self.path_idx          = 0
        self.future            = None

        self.clt_plan_path = self.create_client(
            GetPlan, '/path_planning/plan_path')
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 1)
        self.sub_stop    = self.create_subscription(
            Bool, '/stop_exploration', self.callback_stop, 1)

        self.mission_timer = self.create_timer(0.1, self.mission_loop_callback)


def main(args=None):
    rclpy.init(args=args)
    robot_mission_node = Mission()
    try:
        rclpy.spin(robot_mission_node)
    except KeyboardInterrupt:
        pass
    robot_mission_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()