#!/usr/bin/python3
import rospy
from uuv_gazebo_ros_plugins_msgs.msg import FloatStamped
from nav_msgs.msg import Odometry
import numpy as np
from pybodhi import Tree, Sequence, Fallback

class Targeter(Tree):

    def __init__(self, waypoints):

        # mission
        self.waypoints = waypoints
        self.terminal_waypoint_index = len(self.waypoints) - 1

        # queued waypoints from commander or autonomy
        self.commanded_waypoints = np.empty((0,3))
        self.autonomy_waypoints = np.empty((0,3))

        # behaviour tree
        s1 = Sequence([
            self._mission_not_aborted,
            self._able_to_descend,
            self._actuator_operational,
            self._prop_operational,
            self._no_leaks,
            self._depth_okay
        ])
        s1 = Fallback([s1, Sequence([self._set_mission_abort, self._drop_weight, self._go_to_surface])])
        s2 = Fallback([self._path_obstacle_free, self._avoid_obstacles])
        sp = Sequence([
            Fallback([self._away_from_ship, self._keep_distance]),
            Fallback([self._go_command_received, self._wait_for_go]),
            Fallback([self._compass_calibrated, self._calibrate_compass]),
            Fallback([self._payload_on, self._turn_on_payload]),
            Fallback([self._at_target_depth, self._adjust_depth]),
            Fallback([self._continue_command_received, self._wait_for_continue])
        ])
        sp = Fallback([self._continue_command_received, sp])
        ms = Sequence([
            Fallback([self._no_go_to_surface, self._go_to_surface]),
            Fallback([self._no_commanded_waypoints, self._update_commanded_waypoints]),
            Fallback([self._no_autonomy_waypoints, self._update_autonomy_waypoints])
        ])
        ms = Fallback([self._mission_synchronised, ms])
        me = Fallback([self._at_target_waypoint, self._go_to_target_waypoint])
        me = Sequence([me, self._update_target_waypoint])
        me = Fallback([self._mission_complete, me])
        mf = Sequence([
            Fallback([self._at_surface, self._go_to_surface]),
            Fallback([self._payload_off, self._shutdown_payload])
        ])
        mf = Fallback([self._mission_done, mf])
        Tree.__init__(self, Sequence([s1, s2, sp, ms, me, mf]))

        # safety 1 attributes
        self.mission_not_aborted  = True
        self.able_to_descend      = True
        self.actuator_operational = True
        self.prop_operational     = True
        self.no_leaks             = True
        self.depth_okay           = True

        # system preparation attributes
        self.continue_command_received = False
        self.go_command_received       = False
        self.compass_calibrated        = False
        self.payload_on                = False
        self.target_depth              = self.waypoints[0][2]
        self.start_distance            = 10

        # mission synchronisation attributes
        self.no_go_to_surface_command = True
        self.no_new_waypoints         = True

        # mission execution
        self.target_waypoint_index = 0
        self.mission_complete      = False

        # mission finalisation
        self.payload_off = False

        # call counter
        self.first = True
        self.counter = 0

        # records
        self.positions = list()
        self.headings = list()

        # initialise ros node
        rospy.init_node('porto_bt')

        # left right
        self.fin0 = rospy.Publisher("/lolo_auv_1/fins/0/input", FloatStamped, queue_size=1) # back horizontal -60 to 60
        self.fin1 = rospy.Publisher("/lolo_auv_1/fins/1/input", FloatStamped, queue_size=1) # back horizontal -60 to 60
        self.fin2 = rospy.Publisher("/lolo_auv_1/fins/2/input", FloatStamped, queue_size=1) # back horizontal 60 to -60
        self.fin3 = rospy.Publisher("/lolo_auv_1/fins/3/input", FloatStamped, queue_size=1) # back horizontal 60 to -60

        # down up
        self.bfin = rospy.Publisher("/lolo_auv_1/back_fins/0/input", FloatStamped, queue_size=1) # back vertical 60 to -60
        self.fin4 = rospy.Publisher("/lolo_auv_1/fins/4/input", FloatStamped, queue_size=1)      # back vertical -60 to 60
        self.fin5 = rospy.Publisher("/lolo_auv_1/fins/5/input", FloatStamped, queue_size=1)      # back vertical 60 to -60

        # thrusters
        self.lthrust = rospy.Publisher("/lolo_auv_1/thrusters/0/input", FloatStamped, queue_size=1)
        self.rthrust = rospy.Publisher("/lolo_auv_1/thrusters/1/input", FloatStamped, queue_size=1)

        # stream pose data to controller
        rospy.Subscriber("/lolo_auv_1/pose_gt", Odometry, self.sync)
        rospy.spin()

    def sync(self, data):

        # cartesian position
        self.position = np.array([
            data.pose.pose.position.x,
            data.pose.pose.position.y,
            data.pose.pose.position.z
        ], float)

        # quaternion
        qi = data.pose.pose.orientation.x
        qj = data.pose.pose.orientation.y
        qk = data.pose.pose.orientation.z
        qr = data.pose.pose.orientation.w

        # heading direction
        self.heading = np.array([
            1 - 2*(qj**2 + qk**2),
            2*(qi*qj + qk*qr),
            2*(qi*qk - qj*qr)
        ], float)

        self.positions.append(self.position)
        self.headings.append(self.heading)

        # the starting coordinates
        if self.first == True:
            self.origin = np.copy(self.position)
            self.origin[2] = 0

        self.first = False

        # querry the behaviour tree
        self()

    def pitch(self, sig): # -1 to 1, up to down
        out = FloatStamped()
        out.data = -60*sig
        self.bfin.publish(out)
        self.fin5.publish(out)
        out.data *= -1
        self.fin4.publish(out)

    def yaw(self, sig): # -1 to 1, left to right
        out = FloatStamped()
        out.data = 60*sig
        self.fin0.publish(out)
        self.fin1.publish(out)
        out.data *= -1
        self.fin2.publish(out)
        self.fin3.publish(out)

    def thrust(self, sig): # 0 to 1
        out = FloatStamped()
        out.data = sig*200
        self.lthrust.publish(out)
        self.rthrust.publish(out)

    def control_to_target(self, target):

        # position relative to wp
        pr = target - self.position

        # direction to wp
        dp = pr/np.linalg.norm(pr)

        # planar angle between heading and waypoint directions
        a = np.arctan2(
            self.heading[0]*dp[1] - self.heading[1]*dp[0],
            self.heading[0]*dp[0] + self.heading[1]*dp[1]
        )

        # p control to match pr
        self.yaw(np.clip(-a, -1, 1))

        # p control to match depth
        self.pitch(np.clip(pr[2], -1, 1))

        # full thrust
        self.thrust(1)

    # safety 1
    def _mission_not_aborted(self):
        return int(self.mission_not_aborted)

    def _able_to_descend(self):
        return int(self.able_to_descend)

    def _actuator_operational(self):
        return int(self.actuator_operational)

    def _prop_operational(self):
        return int(self.prop_operational)

    def _no_leaks(self):
        return int(self.no_leaks)

    def _depth_okay(self):
        return int(self.depth_okay)

    def _set_mission_abort(self):
        self._mission_not_aborted = False
        return 1

    def _drop_weight(self):
        return 1

    def _go_to_surface(self):
        # other critical actions in here
        self.control_to_target(self.origin)
        return 2

    # safety 2
    def _path_obstacle_free(self):
        return 1

    def _avoid_obstacles(self):
        return 2

    # system preperation
    def _continue_command_received(self):
        return int(self.continue_command_received)

    def _away_from_ship(self):
        # if not at least start distance
        if np.linalg.norm(self.position - self.origin) < self.start_distance:
            return 0
        else:
            self.thrust(0) # turn off thruster
            return 1

    def _go_command_received(self):
        return int(self.go_command_received)

    def _compass_calibrated(self):
        return int(self.compass_calibrated)

    def _payload_on(self):
        return int(self.payload_on)

    def _at_target_depth(self):
        if abs(self.position[2] - self.waypoints[self.target_waypoint_index][2]) > 1:
            return 0
        else:
            self.thrust(0) # turn off thrusters
            return 1

    def _keep_distance(self):
        # gain distance in direction of first waypoint at original depth
        target = np.copy(self.waypoints[self.target_waypoint_index])
        target[2] = self.origin[2]
        self.control_to_target(target)
        return 2

    def _wait_for_continue(self):
        self.counter += 1
        if self.counter > 100:
            self.continue_command_received = True
            self.counter = 0
        return 2

    def _wait_for_go(self):
        self.counter += 1
        if self.counter > 100:
            self.go_command_received = True
            self.counter = 0
        return 2

    def _calibrate_compass(self):
        self.counter += 1
        if self.counter > 100:
            self.compass_calibrated = True
            self.counter = 0
        return 2

    def _turn_on_payload(self):
        self.counter += 1
        if self.counter > 100:
            self.payload_on = True
            self.counter = 0
        return 2

    def _adjust_depth(self):
        self.control_to_target(self.waypoints[self.target_waypoint_index])
        return 2

    # mission synchronisation
    def _mission_synchronised(self):
        if self.no_go_to_surface_command and self._no_commanded_waypoints and self._no_autonomy_waypoints:
            return 1
        else:
            return 0

    def _no_go_to_surface(self):
        return int(self.no_go_to_surface_command)

    def _no_commanded_waypoints(self):
        if len(self.commanded_waypoints > 0):
            return 0
        else:
            return 1

    def _no_autonomy_waypoints(self):
        if len(self.autonomy_waypoints > 0):
            return 0
        else:
            return 1

    def _update_commanded_waypoints(self):
        return 2

    def _update_autonomy_waypoints(self):
        return 2

    # mission_execution
    def _mission_complete(self):
        if self.target_waypoint_index == self.terminal_waypoint_index:
            return 1
        else:
            return 0

    def _at_target_waypoint(self):
        if np.linalg.norm(self.position - self.waypoints[self.target_waypoint_index]) < 1:
            self.thrust(0)
            return 1
        else:
            return 0

    def _go_to_target_waypoint(self):
        self.control_to_target(self.waypoints[self.target_waypoint_index])
        return 2

    def _update_target_waypoint(self):
        self.target_waypoint_index += 1
        return 2

    # mission_finalisation
    def _mission_done(self):
        if bool(self._at_surface()) and self._payload_off:
            self.thrust(0)
            rospy.signal_shutdown("This shit works")
            return 1
        else:
            return 0

    def _at_surface(self):
        if abs(self.position[2]) < 0.2:
            return 1
        else:
            return 0

    def _payload_off(self):
        return int(self.payload_off)

    def _shutdown_payload(self):
        self.counter += 1
        if self.counter > 100:
            self.payload_off = True
            self.counter = 0
        return 2

if __name__ == "__main__":

    # waypoint sequence
    wps = np.array([
        [100, 100, -10],
        [100, -100, -5],
        [-100, -100, -5]
    ])

    # instantiate behaviour tree
    bt = Targeter(wps)
