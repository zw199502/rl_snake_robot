'''pytest ROS Node'''
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from gazebo_msgs.msg import ModelStates
from sensor_msgs.msg import JointState
from gazebo_msgs.msg import ContactsState
from std_msgs.msg import Float64
import numpy as np

N = 16

class Gazebo_data:
    def __init__(self):
        # rospy.init_node('subsriber_node', anonymous=True)
        # model subscriber, including gplane, pipeline, and snake2
        self.model_subscriber = rospy.Subscriber("/gazebo/model_states", ModelStates, self.model_states_subscriber_callback)
        # joint subsriber, from joint1 to joint16
        self.joint_subscriber = rospy.Subscriber("/snake2/joint_states", JointState, self.joint_states_subscriber_callback)

        # pose of the head
        self.head_orientation = np.zeros((4, 1), dtype=np.float64)
        self.head_position = np.zeros((3, 1), dtype=np.float64)

        # angle of the joint
        self.joint_position = np.zeros((N, 1), dtype=np.float64)
        self.joint_effort = np.zeros((N, 1), dtype=np.float64)
        # effort of the joint

        # spin() simply keeps python from exiting until this node is stopped
        # rospy.spin()




    def model_states_subscriber_callback(self, model):
        #pose of the head
        self.head_orientation[0] = model.pose[2].orientation.x
        self.head_orientation[1] = model.pose[2].orientation.y
        self.head_orientation[2] = model.pose[2].orientation.z
        self.head_orientation[3] = model.pose[2].orientation.w

        self.head_position[0] = model.pose[2].position.x
        self.head_position[1] = model.pose[2].position.y
        self.head_position[2] = model.pose[2].position.z

    def joint_states_subscriber_callback(self, joint):
        # state of the joint
        self.joint_position[0] = joint.position[0]
        self.joint_position[1] = joint.position[8]
        self.joint_position[2] = joint.position[9]
        self.joint_position[3] = joint.position[10]
        self.joint_position[4] = joint.position[11]
        self.joint_position[5] = joint.position[12]
        self.joint_position[6] = joint.position[13]
        self.joint_position[7] = joint.position[14]
        self.joint_position[8] = joint.position[15]
        self.joint_position[9] = joint.position[1]
        self.joint_position[10] = joint.position[2]
        self.joint_position[11] = joint.position[3]
        self.joint_position[12] = joint.position[4]
        self.joint_position[13] = joint.position[5]
        self.joint_position[14] = joint.position[6]
        self.joint_position[15] = joint.position[7]

        self.joint_effort[0] = joint.effort[0]
        self.joint_effort[1] = joint.effort[8]
        self.joint_effort[2] = joint.effort[9]
        self.joint_effort[3] = joint.effort[10]
        self.joint_effort[4] = joint.effort[11]
        self.joint_effort[5] = joint.effort[12]
        self.joint_effort[6] = joint.effort[13]
        self.joint_effort[7] = joint.effort[14]
        self.joint_effort[8] = joint.effort[15]
        self.joint_effort[9] = joint.effort[1]
        self.joint_effort[10] = joint.effort[2]
        self.joint_effort[11] = joint.effort[3]
        self.joint_effort[12] = joint.effort[4]
        self.joint_effort[13] = joint.effort[5]
        self.joint_effort[14] = joint.effort[6]
        self.joint_effort[15] = joint.effort[7]

