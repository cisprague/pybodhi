#!/usr/bin/python3
import rospy
from uuv_gazebo_ros_plugins_msgs.msg import FloatStamped
from nav_msgs.msg import Odometry
import numpy as np
from pybodhi import Tree, Sequence, Fallback

class Targeter(Tree):

    def __init__(self, x, y, z):

        # behaviour tree
        s1 = Sequence([
            self.mission_not_aborted,
            self.able_to_descend,
            self.actuator_operational,
            self.prop_operational,
            self.no_leaks,
            self.depth_okay
        ])
        s1 = Fallback([s1, Sequence([
            self.abort_mission, self.drop_weight, self.go_to_surface
        ])])
        s2 = Fallback([self.path_obstacle_free, self.avoid_obstacles])
        sp = Sequence([
            Fallback([self.away_from_ship, self.keep_distance]),
            Fallback([self.go_command_received, self.wait]),
            Fallback([self.compass_calibrated, self.calibrate_compass]),
            Fallback([self.payload_on, self.turn_on_payload]),
            Fallback([self.at_target_depth, self.adjust_depth]),
            Fallback([self.continue_command_received, self.wait])
        ])
        sp = Fallback([self.continue_command_received, sp])
        ms = Sequence([
            Fallback([self.no_go_to_surface, self.go_to_surface]),
            Fallback([self.no_commanded_waypoints, self.update_waypoints]),
            Fallback([self.no_autonomy_waypoints, self.update_waypoints])
        ])
        ms = Fallback([self.mission_synchronised, ms])
        me = Fallback([self.at_target_waypoint, self.go_to_target_waypoint])
        me = Sequence([me, self.update_target_waypoint])
        me = Fallback([self.mission_complete, me])
        mf = Sequence([
            Fallback([self.at_surface, self.go_to_surface]),
            Fallback([self.payload_off, self.shutdown_payload])
        ])
        mf = Fallback([self.mission_done, mf])
        Tree.__init__(self, Sequence([s1, s2, sp, ms, me, mf]))

        # initialise conditionals
        self.mission_not_aborted = True
        self.able_to_descend     = True
        self.actuator_operational = True
        self.prop_operational = True
        self.no_leaks = True
        self.depth_okay = True

        # initialise ros node
        rospy.init_node('porto_bt')

        # target waypoints
        self.set_wp(x, y, z)

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



    def set_wp(self, x, y, z):
        self.wp = np.array([x, y, z], float)

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

    def control_target_waypoint(self, data):

        # cartesian position
        p = np.array([
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
        dh = np.array([
            1 - 2*(qj**2 + qk**2),
            2*(qi*qj + qk*qr),
            2*(qi*qk - qj*qr)
        ], float)

        # position relative to wp
        pr = self.wp - p

        # direction to wp
        dp = pr/np.linalg.norm(pr)

        # planar angle between heading and waypoint directions
        a = np.arctan2(
            dh[0]*dp[1] - dh[1]*dp[0],
            dh[0]*dp[0] + dh[1]*dp[1]
        )

        # p control to match pr
        self.yaw(np.clip(-a, -1, 1))

        # p control to match depth
        self.pitch(np.clip(pr[2], -1, 1))

        # full thrust
        self.thrust(1)
        print(p)
        if np.linalg.norm(pr) < 1.5:
            print("Hooooray!")

    # safety 1
    def mission_not_aborted(self):
        return 1

    def able_to_descend(self):
        return 1

    def actuator_operational(self):
        return 1

    def prop_operational(self):
        return 1

    def no_leaks(self):
        return 1

    def depth_okay(self):
        return 1

    def abort_mission(self):
        return None

    def drop_weight(self):
        return None

    def go_to_surface(self):
        return None

    # safety 2
    def path_obstacle_free(self):
        return 1

    def avoid_obstacles(self):
        return None

    # system preperation
    def continue_command_received(self):
        return self.continue_command_received

    def away_from_ship(self):
        return None

    def go_command_received(self):
        return None

    def compass_calibrated(self):
        return None

    def payload_on(self):
        return None

    def at_target_depth(self):
        return None

    def keep_distance(self):
        return None

    def wait_for_continue(self):
        return None

    def wait_for_go(self):
        return None

    def wait(self):
        return None

    def calibrate_compass(self):
        return None

    def turn_on_payload(self):
        return None

    def adjust_depth(self):
        return None

    # mission synchronisation
    def mission_synchronised(self):
        return None

    def no_go_to_surface(self):
        return None

    def no_commanded_waypoints(self):
        return None

    def no_autonomy_waypoints(self):
        return None

    def update_waypoints(self):
        return None

    # mission_execution
    def mission_complete(self):
        return None

    def at_target_waypoint(self):
        return None

    def go_to_target_waypoint(self):
        return None

    def update_target_waypoint(self):
        return None

    # mission_finalisation
    def mission_done(self):
        return None

    def at_surface(self):
        return None

    def payload_off(self):
        return None

    def shutdown_payload(self):
        return None

if __name__ == "__main__":

    Targeter(50, 50, -10)
