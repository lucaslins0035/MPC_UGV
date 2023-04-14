#!/usr/bin/env python

import threading
import uuid
import rospy
import actionlib
import do_mpc
import casadi as cdi
import numpy as np
import math
import tf2_ros
from ackermann_msgs.msg import AckermannDrive
from nav_msgs.msg import Path, OccupancyGrid
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion

from gem_interfaces.msg import TrackPathAction, TrackPathResult


class TrackPathServer:
    def __init__(self):
        self.init_tf = None
        # self.center_footprint_tf = None
        self.stop_getting_tf = False
        self.path_tracking_error = []
        self.path_index = 0

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.tf_lock = threading.Lock()
        self.tf_thread = threading.Thread(target=self.get_tf)

        self.ackerman_pub = rospy.Publisher(
            'ackermann_cmd', AckermannDrive, queue_size=10)

        self.local_plan_pub = rospy.Publisher(
            'local_plan', Path, queue_size=10)

        self.costmap = None
        self.costmap_res = None
        self.costmap_size = None
        self.costmap_size_px = None
        self.costmap_pub = rospy.Subscriber(
            '/costmap_node/costmap/costmap', OccupancyGrid, self.costmap_cb)

        self.server = actionlib.SimpleActionServer(
            'track_path', TrackPathAction, self.execute_action, False)
        self.server.start()

        rospy.sleep(2.0)
        rospy.loginfo("Server initialized")

    # def get_cell_cost(self, x, y):
    #     diff_x = x - self.costmap_orig[0]
    #     diff_y = y - self.costmap_orig[1]

    #     if diff_x < self.costmap_size[0] and \
    #             diff_y < self.costmap_size[1]:
    #         return float(self.costmap[int((self.costmap_size[1]-diff_y)/self.costmap_res),
    #                                   int(diff_x/self.costmap_res)])

    #     return 0.0

    def get_cell_position(self, row, col):
        return [(col*self.costmap_res + self.costmap_orig[0],
                 (self.costmap_size[1] - row*self.costmap_res) + self.costmap_orig[1])]

    def costmap_cb(self, msg):
        if self.costmap is None:
            self.costmap_res = msg.info.resolution
            self.costmap_size = (msg.info.width*self.costmap_res/2,
                                 msg.info.height*self.costmap_res)  # Ref: (x,y) ROS

            self.costmap_size_px = (msg.info.height,
                                    msg.info.width//2)  # Ref: (rows, cols) array

            rospy.loginfo("Res "+str(self.costmap_res))
            rospy.loginfo("Size "+str(self.costmap_size))

        rospy.loginfo("Update")

        self.costmap = np.array(msg.data).reshape(msg.info.height,
                                                  msg.info.width)[:,self.costmap_size_px[1]:]
        self.costmap_orig = [msg.info.origin.position.x + self.costmap_size[0],
                             msg.info.origin.position.y]

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

            # try:
            #     trans = self.tfBuffer.lookup_transform(
            #         'odom', 'center_footprint', rospy.Time())
            # except:
            #     rospy.logwarn_throttle(
            #         2.0,
            #         "Failed to find TF from 'odom' to 'center_footprint'")
            # else:
            #     self.center_footprint_tf = np.array([
            #         trans.transform.translation.x,
            #         trans.transform.translation.y,
            #         euler_from_quaternion((trans.transform.rotation.x,
            #                                trans.transform.rotation.y,
            #                                trans.transform.rotation.z,
            #                                trans.transform.rotation.w))[2]]).reshape(-1, 1)
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
        self.tvp_costmap = self.model.set_variable(
            var_type='_tvp', var_name='costmap', shape=(self.costmap_size_px[0],
                                                        self.costmap_size_px[1]))
        self.tvp_cell_pos_x = self.model.set_variable(
            var_type='_tvp', var_name='cell_pos_x', shape=(self.costmap_size_px[0],
                                                           self.costmap_size_px[1]))
        self.tvp_cell_pos_y = self.model.set_variable(
            var_type='_tvp', var_name='cell_pos_y', shape=(self.costmap_size_px[0],
                                                           self.costmap_size_px[1]))

        # Set right-hand side
        self.model.set_rhs(
            'x', self.input_v*cdi.cos(self.state_theta + cdi.atan(cdi.tan(self.input_psi)/2)))
        self.model.set_rhs(
            'y', self.input_v*cdi.sin(self.state_theta + cdi.atan(cdi.tan(self.input_psi)/2)))
        self.model.set_rhs('theta', self.input_v*cdi.tan(self.input_psi)
                           * cdi.cos(cdi.atan(cdi.tan(self.input_psi)/2)))
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
            't_step': 0.5,
            'store_full_solution': True,
            'nlpsol_opts': {'ipopt.print_level': 0, 'ipopt.sb': 'yes', 'print_time': 0}
        }
        self.mpc.set_param(**setup_mpc)

        # cdi.Function('costmap_cost', [self.state_x, self.state_y],)

        # Objective function #TODO? + 1*(theta - theta_set)**2
        lterm = 1e-2 * (self.state_x - self.tvp_x_set)**2 + 1e-2 * \
            (self.state_y - self.tvp_y_set)**2
        mterm = lterm

        self.mpc.set_objective(mterm=mterm, lterm=lterm)

        self.mpc.set_rterm(
            v=1e2,
            psi=1e0)

        # Parameters
        self.mpc.set_uncertainty_values(l=[1.75], car_radius=[1.5])
        # p_template = self.mpc.get_p_template(1)
        # p_template['_p', 0] = np.array([1.75, 0.82])

        # def p(t_now):
        #     return p_template

        # self.mpc.set_p_fun(p)

        # Time-varying Parameters
        tvp_template = self.mpc.get_tvp_template()

        self.path_x_cp = [
            pose.pose.position.x for pose in action_goal.path.poses]
        self.path_y_cp = [
            pose.pose.position.y for pose in action_goal.path.poses]

        def tvp_fun(t_now):
            tvp_template['_tvp', :,
                         'x_set'] = self.path_x_cp[self.path_index]
            tvp_template['_tvp', :,
                         'y_set'] = self.path_y_cp[self.path_index]


            tvp_template['_tvp', :,
                         'costmap'] = self.costmap


            cell_pos = np.zeros(
                (self.costmap_size_px[0], self.costmap_size_px[1], 2), dtype=np.float32)
            for i in np.arange(self.costmap_size_px[0]):
                for j in np.arange(self.costmap_size_px[1]):
                    cell_pos[i, j] = np.array(self.get_cell_position(i, j))


            tvp_template['_tvp', :, 'cell_pos_x'] = cell_pos[:, :, 0]
            tvp_template['_tvp', :, 'cell_pos_y'] = cell_pos[:, :, 1]


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
        #TODO WAY TO EXPENSIVE!!!
        for i in np.arange(1, self.costmap_size_px[0]-1):
            for j in np.arange(1, self.costmap_size_px[1]-1):
                self.mpc.set_nl_cons(
                    str(uuid.uuid4()),
                    cdi.if_else(cdi.logic_and(self.tvp_costmap[i, j] > 90,
                                              cdi.logic_not(
                        cdi.logic_and(self.tvp_costmap[i-1, j] > 90,
                                      cdi.logic_and(self.tvp_costmap[i+1, j] > 90,
                                                    cdi.logic_and(self.tvp_costmap[i, j-1] > 90,
                                                                  self.tvp_costmap[i, j+1] > 90))))),
                                1.5**2 - (self.state_x-self.tvp_cell_pos_x[i, j])**2 - (
                        self.state_y-self.tvp_cell_pos_y[i, j])**2,
                        cdi.SX(0.0)),
                    ub=0.0,
                    soft_constraint=True)


        self.mpc.setup()
        rospy.loginfo("MPC setup finish successfully!")

    def execute_action(self, goal):
        rospy.loginfo("Goal received")

        # Start getting TF data in the background
        self.stop_getting_tf = False
        self.tf_thread.start()

        while self.costmap is None:
            rospy.logwarn_throttle(
                2.0, "Waiting for a costmap msg.")

        try:
            self.setup_controller(goal)
        except Exception as e:
            rospy.logerr(str(e))
            rospy.logerr("Failed setting up Controller! Aborting...")
            self.server.set_aborted(TrackPathResult(result=1))
            self.stop_getting_tf = True
            self.tf_thread.join()
            self.init_tf = None
            return  # TODO Necessary?

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
        self.path_tracking_error = []

        local_plan = Path()
        local_plan.header.frame_id = 'odom'

        # Run control loop until the robot is less then 0.25 m away from the last pose of the path
        rospy.loginfo("Starting control loop")
        while not rospy.is_shutdown() and \
                self.path_index <= len(self.path_x_cp)-1 and \
                self.path_index <= len(self.path_y_cp)-1 and \
                math.sqrt((x0[0, 0]-self.path_x_cp[-1])**2 + (x0[1, 0]-self.path_y_cp[-1])**2) > 0.5:

            start_loop_time = rospy.get_rostime()

            # Remove reached or passed path points
            while math.sqrt((x0[0, 0]-self.path_x_cp[self.path_index])**2 + (x0[1, 0]-self.path_y_cp[self.path_index])**2) < 2.0 and \
                    self.path_index < len(self.path_x_cp)-1 and \
                    self.path_index < len(self.path_y_cp)-1:
                self.path_index = self.path_index + 1

            # Run the optimizer to find the next control effort
            t = rospy.get_rostime()
            u0 = self.mpc.make_step(x0)
            rospy.loginfo("MPC time: {} ms".format(
                (rospy.get_rostime() - t).to_sec()*1000))
            
            rospy.loginfo("Control {} {}".format(u0[0], u0[1]))

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
                self.path_tracking_error.append(dist)
                rospy.loginfo_throttle(
                    1.0, "Path tracking error: {}".format(dist))

            # Publish local plan
            preds_x = self.mpc.data.prediction(('_x', 'x'))[0, :, 0]
            preds_y = self.mpc.data.prediction(('_x', 'y'))[0, :, 0]
            local_plan.poses.clear()
            for i in np.arange(len(preds_x)):
                pose = PoseStamped()
                pose.pose.position.x = preds_x[i]
                pose.pose.position.y = preds_y[i]
                local_plan.poses.append(pose)
            self.local_plan_pub.publish(local_plan)

            elapsed = (rospy.get_rostime()-start_loop_time).to_sec()
            if elapsed > self.mpc.t_step:
                rospy.logwarn("Control loop failed to meet period of {}. Took {}".format(
                    self.mpc.t_step, elapsed))

            control_rate.sleep()

            # Get next state
            x0 = self.init_tf

        # Stop the robot
        control_msg = AckermannDrive()
        self.ackerman_pub.publish(control_msg)

        self.server.set_succeeded(
            TrackPathResult(result=0,
                            path_tracking_error=self.path_tracking_error))
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
