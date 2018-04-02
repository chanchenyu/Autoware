import cv2
import rospy
import numpy as np
import tf
import scipy.io 
from std_msgs.msg import String, Header
from geometry_msgs.msg import PoseStamped, TwistStamped, Pose2D
from path_follower.msg import TrajectoryPoint2D,Trajectory2D, ApplanixPose, traj_plan

des_traj_x = 0
des_traj_y = 0
s_next_x = 0
s_next_y = 0
states_x = 0
states_y = 0

def predictionCallback(data):
	global s_next_x, s_next_y
	s_next_x = data.smooth_x
	s_next_y = data.smooth_y


def desireCallback(data):
	global des_traj_x, des_traj_y
	des_traj_x = data.x
	des_traj_y = data.y


def stateCallback(data):
	global states_x, states_y
	states_x = data.smooth_x
	states_y = data.smooth_y


def plot():
	rospy.init_node('plot', anonymous=True)
	# set node rate
	loop_rate = 100
	dt        = 1.0 / loop_rate
	rate      = rospy.Rate(loop_rate)

	# topic subscriptions / publications
	rospy.Subscriber("s_next_", ApplanixPose, predictionCallback)
	rospy.Subscriber("des_traj_", traj_plan, desireCallback)
	rospy.Subscriber("Applanix_", ApplanixPose, stateCallback)


	img = np.zeros((512,512,3), np.uint8)
	cv2.namedWindow("preview")
	cv2.circle(img, (states_x, states_y), 3, (0, 255, 0), -1)
	print("states_x=",states_x)
	cv2.circle(img, (des_traj_x, des_traj_y), 3, (0, 255, 255), -1)
	cv2.imshow('preview', img)
	cv2.waitKey()

	rate.sleep()


if __name__ == '__main__':
	try:
		print "waypoints_from_mat node starts"
		plot()
	except rospy.ROSInterruptException:
		pass
