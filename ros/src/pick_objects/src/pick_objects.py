#!/usr/bin/env python

'''
Ideas taken from https://github.com/markwsilliman/turtlebot/blob/master/go_to_specific_point_on_map.py
'''

import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion

class GoToPose():
    def __init__(self):

        self.goal_sent = False

	rospy.on_shutdown(self.shutdown)
	
	self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
	rospy.loginfo("Wait for the action server to come up")

	self.move_base.wait_for_server(rospy.Duration(10))

    def goto(self, pos, quat):

        # Send a goal
        self.goal_sent = True
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(Point(pos['x'], pos['y'], 0.000),
                                     Quaternion(quat['r1'], quat['r2'], quat['r3'], quat['r4']))

	    # Start moving
        self.move_base.send_goal(goal)

        success = self.move_base.wait_for_result() # rospy.Duration(60)

        state = self.move_base.get_state()
        result = False

        if success and state == GoalStatus.SUCCEEDED:
            result = True
        else:
            print "cancelling..."
            print success
            print state
            self.move_base.cancel_goal()

        self.goal_sent = False
        return result

    def shutdown(self):
        if self.goal_sent:
            self.move_base.cancel_goal()
        rospy.loginfo("Stop")
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        rospy.init_node('pick_objects', anonymous=False)
        navigator = GoToPose()

        position = {'x': -6.74, 'y' : 1.35}
        position = {'x': -6.44562959671, 'y' : 1.26055216789}
        quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000}

        rospy.loginfo("Sending Pick Up Goal (%s, %s)", position['x'], position['y'])

        pickup_zone_reached = navigator.goto(position, quaternion)

        if pickup_zone_reached is True:
            rospy.loginfo("Reach pickup zone, waiting 5 seconds.")
        else:
            rospy.loginfo("Failed to reach pickup zone.")
            quit()

        rospy.sleep(5)

        position = {'x': -6.9, 'y' : -5.99}
        position = {'x': 0.128987312317, 'y' : -1.71386241913}

        rospy.loginfo("Sending Drop-off Goal (%s, %s)", position['x'], position['y'])

        dropoff_zone_reached = navigator.goto(position, quaternion)

        if dropoff_zone_reached is True:
            rospy.loginfo("Drop off zone reached.")
        else:
            rospy.loginfo("Drop-off Failed.")

    except rospy.ROSInterruptException:
        rospy.loginfo("Ctrl-C caught. Quitting")

