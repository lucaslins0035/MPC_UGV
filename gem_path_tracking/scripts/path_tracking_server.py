#!/usr/bin/env python

import threading
import rospy
import actionlib
import do_mpc
import casadi as cdi
import numpy as np
import math
import tf2_ros
from ackermann_msgs.msg import AckermannDrive
from tf.transformations import euler_from_quaternion

from gem_interfaces.msg import TrackPathAction


class TrackPathServer:
    def __init__(self):
        self.init_tf = None
        self.stop_getting_tf = False
        self.path_tracking_accuracy = []
        self.path_index = 0

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.tf_lock = threading.Lock()
        self.tf_thread = threading.Thread(target=self.get_tf)

        self.ackerman_pub = rospy.Publisher(
            'ackermann_cmd', AckermannDrive, queue_size=10)

        self.server = actionlib.SimpleActionServer(
            'track_path', TrackPathAction, self.execute_action, False)
        self.server.start()

        rospy.sleep(2.0)
        rospy.loginfo("Server initialized")

    def get_tf(self):
        tf_rate = rospy.Rate(10)
        while not self.stop_getting_tf:
            try:
                trans = self.tfBuffer.lookup_transform(
                    'odom', 'base_footprint', rospy.Time())
            except:
                rospy.logwarn_throttle(
                    2.0,
                    "Failed to find TF from 'odom' to 'base_footprint'")
            else:
                self.init_tf = np.array([
                    trans.transform.translation.x,
                    trans.transform.translation.y,
                    euler_from_quaternion((trans.transform.rotation.x,
                                           trans.transform.rotation.y,
                                           trans.transform.rotation.z,
                                           trans.transform.rotation.w))[2]]).reshape(-1, 1)
            tf_rate.sleep()

    def setup_controller(self, action_goal):
        # Building the model
        self.model = do_mpc.model.Model('continuous')

        # State variables
        self.state_x = self.model.set_variable(
            var_type='_x', var_name='x', shape=(1, 1))
        self.state_y = self.model.set_variable(
            var_type='_x', var_name='y', shape=(1, 1))
        self.state_theta = self.model.set_variable(
            var_type='_x', var_name='theta', shape=(1, 1))
        # psi = self.model.set_variable(var_type='_x', var_name='psi', shape=(1, 1))

        # Input variables
        self.input_v = self.model.set_variable(
            var_type='_u', var_name='v', shape=(1, 1))
        self.input_psi = self.model.set_variable(
            var_type='_u', var_name='psi', shape=(1, 1))
        # psi_dot = self.model.set_variable(var_type='_u', var_name='psi_dot', shape=(1, 1))

        # Sys parameters
        self.param_l = self.model.set_variable(
            var_type='_p', var_name='l', shape=(1, 1))
        self.param_car_radius = self.model.set_variable(
            var_type='_p', var_name='car_radius', shape=(1, 1))

        # Time-varying parameters
        self.tvp_x_set = self.model.set_variable(
            var_type='_tvp', var_name='x_set', shape=(1, 1))
        self.tvp_y_set = self.model.set_variable(
            var_type='_tvp', var_name='y_set', shape=(1, 1))

        # Set right-hand side
        self.model.set_rhs('x', self.input_v*cdi.cos(self.state_theta))
        self.model.set_rhs('y', self.input_v*cdi.sin(self.state_theta))
        self.model.set_rhs('theta', self.input_v *
                           cdi.tan(self.input_psi)/self.param_l)
        # self.model.set_rhs('psi', psi_dot)

        self.model.setup()
        rospy.loginfo("Model setup finish successfully!")

        # ==========================================================
        # Setting MPC
        self.mpc = do_mpc.controller.MPC(self.model)

        # Optimizer
        n_horizon = 20
        setup_mpc = {
            'n_horizon': n_horizon,
            't_step': 0.1,
            'store_full_solution': True,
            'nlpsol_opts': {'ipopt.print_level': 0, 'ipopt.sb': 'yes', 'print_time': 0}
        }
        self.mpc.set_param(**setup_mpc)

        # Objective function
        lterm = 1e-2 * (self.state_x - self.tvp_x_set)**2 + 1e-2 * \
            (self.state_y - self.tvp_y_set)**2  # + 1*(theta - 0)**2
        mterm = lterm

        self.mpc.set_objective(mterm=mterm, lterm=lterm)

        self.mpc.set_rterm(
            v=1e2,
            psi=1e0)

        # Parameters
        p_template = self.mpc.get_p_template(1)
        p_template['_p', 0] = np.array([1.75, 0.82])

        def p(t_now):
            return p_template

        self.mpc.set_p_fun(p)

        # Time-varying Parameters
        tvp_template = self.mpc.get_tvp_template()

        self.path_x_cp = [
            pose.pose.position.x for pose in action_goal.path.poses]
        self.path_y_cp = [
            pose.pose.position.y for pose in action_goal.path.poses]

        def tvp_fun(t_now):
            for k in range(n_horizon+1):
                tvp_template['_tvp', k, 'x_set'] = self.path_x_cp[0]
                tvp_template['_tvp', k, 'y_set'] = self.path_y_cp[0]

            return tvp_template

        self.mpc.set_tvp_fun(tvp_fun)

        # Constraints

        # States
        # mpc.bounds['upper', '_x', 'x'] = 10.0
        # mpc.bounds['lower', '_x', 'x'] = -10.0

        # mpc.bounds['upper', '_x', 'y'] = 10.0
        # mpc.bounds['lower', '_x', 'y'] = -10.0

        self.mpc.bounds['upper', '_x', 'theta'] = 2*np.pi
        self.mpc.bounds['lower', '_x', 'theta'] = -2*np.pi

        # mpc.bounds['upper', '_x', 'psi'] = 0.78  # rad
        # mpc.bounds['lower', '_x', 'psi'] = -0.78  # rad

        # Inputs
        self.mpc.bounds['upper', '_u', 'v'] = 1.5  # m/s
        self.mpc.bounds['lower', '_u', 'v'] = -1.5  # m/s

        self.mpc.bounds['upper', '_u', 'psi'] = 0.61  # rad
        self.mpc.bounds['lower', '_u', 'psi'] = -0.61  # rad

        # mpc.bounds['upper', '_u', 'psi_dot'] = 10.0  # rad/s
        # mpc.bounds['lower', '_u', 'psi_dot'] = -10.0  # rad/s

        # NL Constraints

        # circles = [
        #     [1.0, 1.8, 0.2],
        #     [5.0, 3.4, 0.2],
        #     # [5.1, 5.5, 0.1],
        # ]

        # for i, circle in enumerate(circles):
        #     self.mpc.set_nl_cons('obstacle'+str(i), (self.param_car_radius+circle[2]) - cdi.sqrt(
        #         (self.state_x-circle[0])**2 + (self.state_y-circle[1])**2), ub=0.0, soft_constraint=True)

        self.mpc.setup()
        rospy.loginfo("MPC setup finish successfully!")

    def execute_action(self, goal):
        rospy.loginfo("Goal received")

        # Start getting TF data in the background
        self.stop_getting_tf = False
        self.tf_thread.start()

        try:
            self.setup_controller(goal)
        except:
            rospy.logerr("Failed setting up Controller! Aborting...")
            self.server.set_aborted()
            self.stop_getting_tf = True
            self.tf_thread.join()
            self.init_tf = None
            return  # TODO Necessary?

        # TODO check finihsing process
        while self.init_tf is None:
            rospy.logwarn_throttle(
                2.0, "Waiting for a TF from 'odom' to 'base_footprint' to became available.")

        # Get initial state
        x0 = self.init_tf

        self.mpc.x0 = x0
        self.mpc.set_initial_guess()

        self.mpc.reset_history()

        control_rate = rospy.Rate(1/self.mpc.t_step)

        self.path_index = 0
        self.path_tracking_accuracy = []

        # Run control loop until the robot is less then 0.25 m away from the last pose of the path
        rospy.loginfo("Starting control loop")
        while not rospy.is_shutdown() and \
                len(self.path_x_cp) > 1 and \
                len(self.path_y_cp) > 1 and \
                math.sqrt((x0[0, 0]-self.path_x_cp[-1])**2 + (x0[1, 0]-self.path_y_cp[-1])**2) > 0.25:

            start_loop_time = rospy.get_rostime()

            # Remove reached or passed path points
            while math.sqrt((x0[0, 0]-self.path_x_cp[0])**2 + (x0[1, 0]-self.path_y_cp[0])**2) < 2.0 and \
                    len(self.path_x_cp) > 1 and \
                    len(self.path_y_cp) > 1:
                self.path_x_cp.pop(0)
                self.path_y_cp.pop(0)
                self.path_index = self.path_index + 1

            # Run the optimizer to find the next control effort
            t = rospy.get_rostime()
            u0 = self.mpc.make_step(x0)
            rospy.loginfo("MPC time: {} ms".format(
                (rospy.get_rostime() - t).to_sec()*1000))

            # Publish control effort
            control_msg = AckermannDrive()
            control_msg.speed = u0[0]
            control_msg.steering_angle = u0[1]
            self.ackerman_pub.publish(control_msg)

            # Calculate robot distance to path
            if self.path_index >= 1:
                dist = abs(
                    (goal.path.poses[self.path_index].pose.position.x -
                     goal.path.poses[self.path_index-1].pose.position.x) *
                    (goal.path.poses[self.path_index-1].pose.position.y - x0[1, 0]) -
                    (goal.path.poses[self.path_index-1].pose.position.x - x0[0, 0]) *
                    (goal.path.poses[self.path_index].pose.position.y -
                     goal.path.poses[self.path_index-1].pose.position.y)
                ) / math.sqrt((goal.path.poses[self.path_index].pose.position.x -
                               goal.path.poses[self.path_index-1].pose.position.x)**2 +
                              (goal.path.poses[self.path_index].pose.position.y -
                               goal.path.poses[self.path_index-1].pose.position.y)**2)
                self.path_tracking_accuracy.append(dist)
                rospy.loginfo_throttle(
                    1.0, "Path tracking error: {}".format(dist))

            elapsed = (rospy.get_rostime()-start_loop_time).to_sec()
            if elapsed > self.mpc.t_step:
                rospy.logwarn_throttle(1.0, "Control loop failed to meet period of {}. Took {}".format(
                    self.mpc.t_step, elapsed))

            control_rate.sleep()

            # Get next state
            x0 = self.init_tf

        self.server.set_succeeded()
        self.stop_getting_tf = True
        self.tf_thread.join()
        self.init_tf = None
        rospy.loginfo("Goal finished successfully!")


if __name__ == '__main__':
    rospy.init_node('track_path_server')
    server = TrackPathServer()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        if server.tf_thread.isAlive():
            server.stop_getting_tf = True
            server.tf_thread.join()
