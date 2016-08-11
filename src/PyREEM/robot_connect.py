import rospy
import actionlib
from sensor_msgs.msg import JointState
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from pal_interaction_msgs.msg import TtsAction, TtsGoal
from pal_web_msgs.msg import WebGoTo
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from sb04_led.srv import SetColorEffect

joint_states = {}

arm_left = [None, set(['arm_left_1_joint', 'arm_left_2_joint', 'arm_left_3_joint', 'arm_left_4_joint', 'arm_left_5_joint', 'arm_left_6_joint', 'arm_left_7_joint'])]

arm_right = [None, set(['arm_right_1_joint', 'arm_right_2_joint', 'arm_right_3_joint', 'arm_right_4_joint', 'arm_right_5_joint', 'arm_right_6_joint', 'arm_right_7_joint'])]

head = [None, set(['head_1_joint', 'head_2_joint'])]

torso = [None, set(['torso_1_joint', 'torso_2_joint'])]

joints = [arm_left, arm_right, head, torso]

tts = None
leds = None
web = None
actions = None

def _arm_controller_updates(msg):
	pass	

def _joint_state_update(msg):
	for i in range(len(msg.name)):
		joint_states[msg.name[i]] = msg.position[i]

def open_web(url):
	msg = WebGoTo()
	msg.type = WebGoTo.URI
	msg.value = url
	web.publish(msg)

def play_motion(name, callback = None):
	goal = PlayMotionGoal()
	goal.skip_planning = False
	goal.motion_name = name
	goal.priority = 20
	actions.send_goal(goal, callback)

def do_trajectory(steps, callback = None):
	all_joints = set()
	for step in steps:
		all_joints.update(step.keys())
	for client, names in joints:
		if not names.isdisjoint(all_joints):
			goal = FollowJointTrajectoryGoal()
			goal.trajectory.joint_names = names
			points = []
			time_from_start = 0
			for step in steps:
				point = JointTrajectoryPoint()
				point.positions = [step[name] if name in step else joint_states[name] for name in names]
				time_from_start += step['time_to_reach']
				point.time_from_start = rospy.Duration(time_from_start)
				points.append(point)
			goal.trajectory.points = points
			client.send_goal(goal, callback)

def set_leds(r, g, b, left=True, right=True):
	leds(([1] if left else []) + ([2] if right else []),r,g,b)

def say(text):
	goal = TtsGoal()
	goal.rawtext.text = text
	goal.rawtext.lang_id = 'en_GB'
	tts.send_goal(goal)

def start():
	global tts, leds, web, actions
	rospy.init_node('PyREEMPy')
	rospy.Subscriber('/joint_states', JointState, _joint_state_update)
	arm_right[0] = actionlib.SimpleActionClient('/right_arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
	arm_left[0] = actionlib.SimpleActionClient('/left_arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
	head[0] = actionlib.SimpleActionClient('/head_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
	torso[0] = actionlib.SimpleActionClient('/torso_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
	actions = actionlib.SimpleActionClient('/play_motion', PlayMotionAction)
	tts = actionlib.SimpleActionClient('/tts', TtsAction)
	leds = rospy.ServiceProxy('/sb04/leds/SetColorEffect', SetColorEffect)
	web = rospy.Publisher('/web/go_to', WebGoTo)


# Unused data

caster_joints = ['caster_left_1_joint', 'caster_left_2_joint', 'caster_right_1_joint', 'caster_right_2_joint']
hand_left_joints = ['hand_left_index_1_joint', 'hand_left_index_2_joint', 'hand_left_index_3_joint', 'hand_left_index_joint', 'hand_left_middle_1_joint', 'hand_left_middle_2_joint', 'hand_left_middle_3_joint', 'hand_left_middle_joint', 'hand_left_thumb_joint']
hand_right_joints = ['hand_right_index_1_joint', 'hand_right_index_2_joint', 'hand_right_index_3_joint', 'hand_right_index_joint', 'hand_right_middle_1_joint', 'hand_right_middle_2_joint', 'hand_right_middle_3_joint', 'hand_right_middle_joint', 'hand_right_thumb_joint'] 
torso_joints = ['torso_1_joint', 'torso_2_joint']
base_joints = ['wheel_left_joint', 'wheel_right_joint']

