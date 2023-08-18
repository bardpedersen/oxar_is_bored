import rospy
from rosgraph_msgs.msg import Clock

class RosbagSynchronizer:
    def __init__(self, topic_sync):

        rospy.init_node('synchronization_node')
        self.sync_pub = rospy.Publisher(topic_sync, Clock, queue_size=10)
        rospy.Subscriber('/clock_sync', Clock, self.sync_callback)

    def sync_callback(self, msg):
        sync_msg = Clock()
        sync_msg.clock = rospy.Time.now()
        self.sync_pub.publish(sync_msg)

if __name__ == '__main__':
    try:
        synchronizer = RosbagSynchronizer("/clock_sync_right")
    except rospy.ROSInterruptException:
        pass
