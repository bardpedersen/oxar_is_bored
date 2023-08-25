import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseArray
from time import sleep


class Path_follower:
    def __init__(self):
        rospy.init_node('path_follower')

        # Create publishers for arm positions
        self.arm1_move_to_position = rospy.Publisher('arm1position', JointState, queue_size=10)
        self.arm2_move_to_position = rospy.Publisher('arm2position', JointState, queue_size=10)

        # Subscribe to arm positions 
        rospy.Subscriber('arm1_cur_pos', PoseArray, self.arm_pos1_callback)
        rospy.Subscriber('arm2_cur_pos', PoseArray, self.arm_pos2_callback)

        # Get params
        self.path = rospy.get_param('arm_movement')

    def arm_pos1_callback(self, data):
        self.pose1 = data.poses[0].position

    def arm_pos2_callback(self, data):
        self.pose2 = data.poses[0].position

    def are_lists_close(self, list1, list2, tolerance=10):
        if abs(list1.x - list2[0]) > tolerance:
            return True
        if abs(list1.y - list2[1]) > tolerance:
            return True
        if abs(list1.z - list2[2]) > tolerance:
            return True
        else:
            return False
        
    def move_arm(self, left=0, path=None):
        if path != None:
            self.path = path

        # Checks one position at a time from list
        for i in self.path:
            # Prints when the arm has reached the desired position
            rospy.loginfo(f'Going to point: {i}')

            # Create message to arm
            joint_state = JointState()
            joint_state.position = [i[0], i[1], i[2]]
            joint_state.velocity = [0.0]
            joint_state.effort = [0]
            rospy.loginfo(f'Message: {joint_state}')

            if left == 0:   # Arm 1
                self.arm1_move_to_position.publish(joint_state)

                # Checks if the arm has reached the desired position
                while self.are_lists_close(self.pose1, i):
                    pass

            if left == 1:   # Arm 2
                self.arm2_move_to_position.publish(joint_state)

                # Checks if the arm has reached the desired position
                while self.are_lists_close(self.pose2, i):
                    pass

            if left == 2:   # Both arms
                self.arm_posit_pub1.publish(joint_state)
                self.arm_posit_pub2.publish(joint_state)

                # Checks if the arms have reached the desired position
                while self.are_lists_close(self.pose2, i) or self.are_lists_close(self.pose1, i):
                    pass



if __name__ == '__main__':
    follower = Path_follower()
    sleep(1)
    follower.move_arm(0)
