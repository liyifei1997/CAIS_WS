#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Holonomic MPC controller for KUKA KMR iiwa base.
Subscribes: /kmriiwa/base/state/odom
Publishes:  /kmriiwa/base/command/cmd_vel
Requires:   do-mpc 5.x, casadi, numpy, rospy
"""

import rospy
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

import do_mpc
from casadi import cos, sin  # CasADi math

class KMRBaseMPCOmni:
    def __init__(self):
        # ===== ROS params =====
        self.dt = rospy.get_param("~dt", 0.02)     # 50 Hz
        self.N  = rospy.get_param("~N", 15)        # horizon steps

        # Velocity limits
        self.vx_max = rospy.get_param("~vx_max", 0.6)
        self.vx_min = rospy.get_param("~vx_min", -0.6)
        self.vy_max = rospy.get_param("~vy_max", 0.6)
        self.vy_min = rospy.get_param("~vy_min", -0.6)
        self.w_max  = rospy.get_param("~w_max", 1.5)

        # Weights
        self.Q_xy = rospy.get_param("~Q_xy", 10.0)
        self.Q_th = rospy.get_param("~Q_th", 0.5)
        self.R_vx = rospy.get_param("~R_vx", 0.05)
        self.R_vy = rospy.get_param("~R_vy", 0.08)
        self.R_w  = rospy.get_param("~R_w",  0.05)
        self.Rd_vx= rospy.get_param("~Rd_vx",0.02)
        self.Rd_vy= rospy.get_param("~Rd_vy",0.02)
        self.Rd_w = rospy.get_param("~Rd_w", 0.02)

        # Waypoints
        self.waypoints = rospy.get_param(
            "~waypoints",
            [[0.0,0.0],[1.0,0.0],[2.0,0.0],[3.0,0.0],[3.0,0.5],[3.0,1.0],[3.0,1.5]]
        )
        self.wp_tol = rospy.get_param("~waypoint_tol", 0.08)

        # Topics (as in kmriiwa_ros_stack)
        self.topic_odom = rospy.get_param("~odom_topic", "/kmriiwa/base/state/odom")
        self.topic_cmd  = rospy.get_param("~cmd_topic",  "/kmriiwa/base/command/cmd_vel")

        # ===== ROS IO =====
        self.pub_cmd = rospy.Publisher(self.topic_cmd, Twist, queue_size=10)
        rospy.Subscriber(self.topic_odom, Odometry, self.odom_cb)
        self.have_odom = False
        self.x = 0.0; self.y = 0.0; self.th = 0.0
        self.ref_idx = 0

        # ===== Build MPC =====
        self.mpc = self._build_mpc()
        self.mpc.x0 = np.zeros((3,1))
        rospy.loginfo("KMR omni-base MPC ready (dt=%.3f N=%d)", self.dt, self.N)

    # ------------------------------------------------------------------
    def _build_mpc(self):
        model = do_mpc.model.Model('continuous')

        # States
        x  = model.set_variable('_x', 'x')
        y  = model.set_variable('_x', 'y')
        th = model.set_variable('_x', 'th')

        # Inputs
        vx = model.set_variable('_u', 'vx')
        vy = model.set_variable('_u', 'vy')
        w  = model.set_variable('_u', 'w')

        # Time-varying Parameters (N+1 each step)  <-- use '_tvp' in do-mpc 5.x
        px_ref = model.set_variable('_tvp', 'px_ref')
        py_ref = model.set_variable('_tvp', 'py_ref')
        th_ref = model.set_variable('_tvp', 'th_ref')

        # Dynamics (world frame)
        cth = cos(th)
        sth = sin(th)
        model.set_rhs('x',  vx*cth - vy*sth)
        model.set_rhs('y',  vx*sth + vy*cth)
        model.set_rhs('th', w)

        model.setup()

        # MPC controller
        mpc = do_mpc.controller.MPC(model)
        mpc.set_param(n_horizon=self.N, t_step=self.dt, store_full_solution=False)

        # Cost
        mterm = self.Q_xy*((x-px_ref)**2 + (y-py_ref)**2) + self.Q_th*(th-th_ref)**2
        lterm = mterm + self.R_vx*(vx**2) + self.R_vy*(vy**2) + self.R_w*(w**2)
        mpc.set_objective(mterm=mterm, lterm=lterm)
        mpc.set_rterm(vx=self.Rd_vx, vy=self.Rd_vy, w=self.Rd_w)

        # Bounds
        mpc.bounds['lower','_u','vx'] = self.vx_min
        mpc.bounds['upper','_u','vx'] = self.vx_max
        mpc.bounds['lower','_u','vy'] = self.vy_min
        mpc.bounds['upper','_u','vy'] = self.vy_max
        mpc.bounds['lower','_u','w']  = -self.w_max
        mpc.bounds['upper','_u','w']  =  self.w_max

        # ---- TVP registration BEFORE setup() ----
        # runtime caches (N+1,1)
        self._px_cache = np.zeros((self.N+1,1))
        self._py_cache = np.zeros((self.N+1,1))
        self._th_cache = np.zeros((self.N+1,1))

        def tvp_fun(_t):
            # Get a fresh DMStruct template and fill stage-wise
            tvp = mpc.get_tvp_template()
            for k in range(self.N+1):
                tvp['_tvp', k, 'px_ref'] = float(self._px_cache[k,0])
                tvp['_tvp', k, 'py_ref'] = float(self._py_cache[k,0])
                tvp['_tvp', k, 'th_ref'] = float(self._th_cache[k,0])
            return tvp

        mpc.set_tvp_fun(tvp_fun)
        # -----------------------------------------

        mpc.setup()
        return mpc

    # ------------------------------------------------------------------
    def _build_ref(self):
        pts = np.asarray(self.waypoints)
        K = len(pts)
        idxs = [min(self.ref_idx+k, K-1) for k in range(self.N+1)]
        traj = pts[idxs,:]
        # Heading reference toward next point
        ths = []
        for k in range(self.N+1):
            i = idxs[k]; j = min(i+1, K-1)
            dx, dy = pts[j,0]-pts[i,0], pts[j,1]-pts[i,1]
            ths.append(np.arctan2(dy, dx) if abs(dx)+abs(dy)>1e-6 else 0.0)
        return traj[:,0].reshape(-1,1), traj[:,1].reshape(-1,1), np.array(ths).reshape(-1,1)

    def _advance_waypoint(self):
        if self.ref_idx >= len(self.waypoints)-1: return
        gx, gy = self.waypoints[self.ref_idx]
        if np.hypot(self.x-gx, self.y-gy) < self.wp_tol:
            self.ref_idx += 1

    # ------------------------------------------------------------------
    def odom_cb(self, msg: Odometry):
        p = msg.pose.pose
        self.x = p.position.x
        self.y = p.position.y
        q = p.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.th = yaw
        self.have_odom = True

    # ------------------------------------------------------------------
    def spin(self):
        rate = rospy.Rate(int(round(1.0/self.dt)))
        while not rospy.is_shutdown():
            if not self.have_odom:
                rate.sleep(); continue

            # Update reference caches for all horizon steps
            self._advance_waypoint()
            px_ref, py_ref, th_ref = self._build_ref()
            self._px_cache[:] = px_ref
            self._py_cache[:] = py_ref
            self._th_cache[:] = th_ref

            # Current state
            x0 = np.array([[self.x],[self.y],[self.th]])
            self.mpc.x0 = x0

            # Solve MPC
            u = self.mpc.make_step(x0)
            vx_cmd, vy_cmd, w_cmd = float(u[0,0]), float(u[1,0]), float(u[2,0])

            # Publish to ROS
            cmd = Twist()
            cmd.linear.x  = vx_cmd
            cmd.linear.y  = vy_cmd
            cmd.angular.z = w_cmd
            self.pub_cmd.publish(cmd)

            rospy.loginfo_throttle(
                1.0,
                "wp=%d/%d pose=(%.2f,%.2f,%.2f) cmd=(vx=%.2f, vy=%.2f, w=%.2f)",
                self.ref_idx, len(self.waypoints)-1,
                self.x, self.y, self.th, vx_cmd, vy_cmd, w_cmd
            )
            rate.sleep()

# ----------------------------------------------------------------------
if __name__ == "__main__":
    rospy.init_node("kmr_base_mpc_omni")
    KMRBaseMPCOmni().spin()
