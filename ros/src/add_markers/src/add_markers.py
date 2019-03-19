#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose
from visualization_msgs.msg import Marker
from std_msgs.msg import Bool

class Markers():
    def __init__(self):

        self.marker_publish = '/markers/path'
        self.map_frame = 'map'
        self.pick_topic = 'pick_flag'
        self.drop_topic = 'drop_flag'

        self.pick_position = {'x': -6.44562959671, 'y' : 1.26055216789}
        self.drop_position = {'x': 0.128987312317, 'y' : -1.71386241913}

        # Publisher
        self.marker_publisher = rospy.Publisher(self.marker_publish, Marker, queue_size = 10)

        # Subscribers
        self.pick_sub = rospy.Subscriber(self.pick_topic, Bool, self.pick_callback)
        self.drop_sub = rospy.Subscriber(self.drop_topic, Bool, self.drop_callback)

    def pick_callback(self, b):
        ''' pick up flag '''
        if b.data is True:
            rospy.loginfo("Received pick-up marker (%s, %s)", self.pick_position['x'], self.pick_position['y'])
            self.publish_marker(self.pick_position, 1., 0., 0., 1., 1, 0, scale=0.2) # Blue sphere
        else:
            rospy.loginfo("deleting pick-up marker")
            self.delete_marker(1)

    def drop_callback(self, b):
        ''' drop off flag '''
        if b.data is True:
            rospy.loginfo("Received drop-off marker (%s, %s)", self.drop_position['x'], self.drop_position['y'])
            self.publish_marker(self.drop_position, 1., 1., 0., 0., 2, 1, scale=0.2) # Red cube
        else:
            rospy.loginfo("deleting drop-off marker")
            self.delete_marker(2)

    def publish_marker(self, pos, alpha, red, green, blue, marker_id, type, scale=0.1):
        ''' Publish a marker '''

        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = rospy.Time.now()

        marker_type = marker.SPHERE if type == 0 else marker.CUBE

        marker.id = marker_id
        marker.type = marker_type
        marker.action = marker.ADD

        marker.pose.position.x = pos['x']
        marker.pose.position.y = pos['y']
        marker.pose.position.z = 0.
        marker.pose.orientation.x = marker.pose.orientation.y = marker.pose.orientation.z = 0.
        marker.pose.orientation.w = 1.
        marker.scale.x = marker.scale.y = marker.scale.z = scale
        marker.color.a = alpha
        marker.color.r = red
        marker.color.g = green
        marker.color.b = blue

        self.marker_publisher.publish(marker)
        

    def delete_marker(self, marker_id):
        ''' Removes a marker '''
    
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = rospy.Time.now()

        marker.id = marker_id
        marker.action = marker.DELETE
        self.marker_publisher.publish(marker)


if __name__ == '__main__':
    try:
        rospy.init_node('add_markers', anonymous=False)
        m = Markers()

        while not rospy.is_shutdown():
            rospy.spin()

        '''
        # First version
        rospy.sleep(5)

        position = {'x': -6.44562959671, 'y' : 1.26055216789}
        rospy.loginfo("Pick Up marker (%s, %s)", position['x'], position['y'])
        m.publish_marker(position, 1., 0., 0., 1., 1, 0, scale=0.2) # Blue sphere

        rospy.sleep(5)
        m.delete_marker(1)

        rospy.sleep(5)

        position = {'x': 0.128987312317, 'y' : -1.71386241913}
        rospy.loginfo("Drop-off marker (%s, %s)", position['x'], position['y'])
        m.publish_marker(position, 1., 1., 0., 0., 2, 1, scale=0.2) # Red cube
        '''

    except rospy.ROSInterruptException:
        rospy.loginfo("Ctrl-C caught. Quitting")

