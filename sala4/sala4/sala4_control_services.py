import importlib
from functools import partial

import rclpy
import rowan
from crazyflie_interfaces.msg import FullState, Hover
from crazyflie_interfaces.srv import (
    GoTo,
    Land,
    NotifySetpointsStop,
    StartTrajectory,
    Takeoff,
    UploadTrajectory,
)
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Empty

# import BackendRviz from .backend_rviz
# from .backend import *
# from .backend.none import BackendNone
from .crazyflie_sil import (
    CrazyflieSIL,  # your SIL class
    TrajectoryPolynomialPiece,
)
from .sim_data_types import State


class ControlServices(Node):
    def __init__(self):
        super().__init__("sala4_control_services")
        # Turn ROS parameters into a dictionary
        self._ros_parameters = self._param_to_dict(self._parameters)
        self.cfs = {}

        world_tf_name = "world"
        robot_yaml_version = 0

        try:
            robot_yaml_version = self._ros_parameters["fileversion"]
        except KeyError:
            self.get_logger().info(
                "No fileversion found in crazyflies.yaml, assuming version 0"
            )

        robot_data = self._ros_parameters["robots"]

        # Parse robots
        names = []
        initial_states = []
        reference_frames = []
        for cfname in robot_data:
            if robot_data[cfname]["enabled"]:
                type_cf = robot_data[cfname]["type"]
                # do not include virtual objects
                connection = self._ros_parameters["robot_types"][type_cf].get(
                    "connection", "crazyflie"
                )
                if connection == "crazyflie":
                    names.append(cfname)
                    pos = robot_data[cfname]["initial_position"]
                    initial_states.append(State(pos))
                    # Get the current reference frame for the robot
                    reference_frame = world_tf_name
                    if robot_yaml_version >= 3:
                        try:
                            reference_frame = self._ros_parameters["all"][
                                "reference_frame"
                            ]
                        except KeyError:
                            pass
                        try:
                            reference_frame = self._ros_parameters["robot_types"][
                                robot_data[cfname]["type"]
                            ]["reference_frame"]
                        except KeyError:
                            pass
                        try:
                            reference_frame = self._ros_parameters["robots"][cfname][
                                "reference_frame"
                            ]
                        except KeyError:
                            pass
                    reference_frames.append(reference_frame)

        # initialize backend by dynamically loading the module
        backend_name = self._ros_parameters["sim"]["backend"]
        module = importlib.import_module(
            ".backend." + backend_name, package="crazyflie_sim"
        )
        class_ = getattr(module, "Backend")
        self.backend = class_(self, names, initial_states)

        # initialize visualizations by dynamically loading the modules
        self.visualizations = []
        for vis_key in self._ros_parameters["sim"]["visualizations"]:
            if self._ros_parameters["sim"]["visualizations"][vis_key]["enabled"]:
                module = importlib.import_module(
                    ".visualization." + str(vis_key), package="crazyflie_sim"
                )
                class_ = getattr(module, "Visualization")
                if vis_key == "rviz":
                    # special case for rviz, which needs the reference frames
                    vis = class_(
                        self,
                        self._ros_parameters["sim"]["visualizations"][vis_key],
                        names,
                        initial_states,
                        reference_frames,
                    )
                else:
                    vis = class_(
                        self,
                        self._ros_parameters["sim"]["visualizations"][vis_key],
                        names,
                        initial_states,
                    )
                self.visualizations.append(vis)

        controller_name = backend_name = self._ros_parameters["sim"]["controller"]

        # create robot SIL objects
        for name, initial_state in zip(names, initial_states):
            self.cfs[name] = CrazyflieSIL(
                name, initial_state.pos, controller_name, self.backend.time
            )

        for name, _ in self.cfs.items():
            pub = self.create_publisher(
                String,
                name + "/robot_description",
                rclpy.qos.QoSProfile(
                    depth=1, durability=rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL
                ),
            )

            msg = String()
            msg.data = self._ros_parameters["robot_description"].replace("$NAME", name)
            pub.publish(msg)

            self.create_service(
                Empty, name + "/emergency", partial(self._emergency_callback, name=name)
            )
            self.create_service(
                Takeoff, name + "/takeoff", partial(self._takeoff_callback, name=name)
            )
            self.create_service(
                Land, name + "/land", partial(self._land_callback, name=name)
            )
            self.create_service(
                GoTo, name + "/go_to", partial(self._go_to_callback, name=name)
            )
            self.create_service(
                StartTrajectory,
                name + "/start_trajectory",
                partial(self._start_trajectory_callback, name=name),
            )
            self.create_service(
                UploadTrajectory,
                name + "/upload_trajectory",
                partial(self._upload_trajectory_callback, name=name),
            )
            self.create_service(
                NotifySetpointsStop,
                name + "/notify_setpoints_stop",
                partial(self._notify_setpoints_stop_callback, name=name),
            )
            self.create_subscription(
                Twist,
                name + "/cmd_vel_legacy",
                partial(self._cmd_vel_legacy_changed, name=name),
                10,
            )
            self.create_subscription(
                Hover,
                name + "/cmd_hover",
                partial(self._cmd_hover_changed, name=name),
                10,
            )
            self.create_subscription(
                FullState,
                name + "/cmd_full_state",
                partial(self._cmd_full_state_changed, name=name),
                10,
            )

        # Create services for the entire swarm and each individual crazyflie
        self.create_service(Takeoff, "all/takeoff", self._takeoff_callback)
        self.create_service(Land, "all/land", self._land_callback)
        self.create_service(GoTo, "all/go_to", self._go_to_callback)
        self.create_service(
            StartTrajectory, "all/start_trajectory", self._start_trajectory_callback
        )

        # This is the last service to announce.
        # Can be used to check if the server is fully available.
        self.create_service(Empty, "all/emergency", self._emergency_callback)

        # step as fast as possible
        max_dt = (
            0.0
            if "max_dt" not in self._ros_parameters["sim"]
            else self._ros_parameters["sim"]["max_dt"]
        )
        self.timer = self.create_timer(max_dt, self._timer_callback)
        self.is_shutdown = False
        # Timer
        self.timer = self.create_timer(1.0 / self.loop_rate, self.timer_callback)

        self.current_state = None
        self.teleop_cmd = None
        self.get_logger().info("ControlServices (SIL + Gazebo bridge) initialized.")

    # ------------------- Callbacks -------------------

    def fullstate_callback(self, msg: FullState):
        self.teleop_cmd = msg
        self.cf.cmdFullState(
            [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z],
            [msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z],
            [msg.acc.x, msg.acc.y, msg.acc.z],
            msg.pose.orientation.z,
            [msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z],
        )

    def odom_callback(self, msg: Odometry):
        # Update the SIL state to match Gazebo odometry
        pos = [
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
        ]
        vel = [
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z,
        ]
        quat = [
            msg.pose.pose.orientation.w,
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
        ]
        omega = [
            msg.twist.twist.angular.x,
            msg.twist.twist.angular.y,
            msg.twist.twist.angular.z,
        ]
        self.cf.setState(State(pos, vel, quat, omega))

    # ------------------- Services -------------------

    def takeoff_callback(self, request, response):
        self.get_logger().info(
            f"Takeoff requested: height={request.height} m, duration={request.duration.sec} s"
        )
        self.cf.takeoff(request.height, request.duration.sec)
        response.success = True
        return response

    def land_callback(self, request, response):
        self.get_logger().info(
            f"Landing requested: height={request.height} m, duration={request.duration.sec} s"
        )
        self.cf.land(request.height, request.duration.sec)
        response.success = True
        return response

    def goto_callback(self, request, response):
        self.get_logger().info(
            f"GoTo requested: pos=({request.goal.x}, {request.goal.y}, {request.goal.z}), yaw={request.yaw}"
        )
        self.cf.goTo(
            [request.goal.x, request.goal.y, request.goal.z],
            request.yaw,
            request.duration.sec,
        )
        response.success = True
        return response

    # ------------------- Timer loop -------------------

    def timer_callback(self):
        # Get SIL setpoint for current time
        state = self.cf.getSetpoint()
        if state is None:
            return

        twist = Twist()
        twist.linear.x = state.vel[0]
        twist.linear.y = state.vel[1]
        twist.linear.z = state.vel[2]
        # yaw rate approximation
        twist.angular.z = state.omega[2]
        self.publisher_.publish(twist)

    # ------------------- Time helper -------------------

    def get_time(self):
        return self.get_clock().now().nanoseconds * 1e-9


def main(args=None):
    rclpy.init(args=args)
    node = ControlServices()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
