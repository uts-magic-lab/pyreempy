#!/usr/bin/env python

import robot_connect as _bot
import node_comms as _comms
import simple_timer as _timer
from actionlib_msgs.msg import GoalStatus

# Callbacks available
onNodeStatusUpdate = None
onTimer = None
onActionSuccess = None
onMoveArmActionFailed = None
onMoveArmActionSuccess = None

# Initialize the robot
def start():
	_bot.start()
	_comms.start()

	def status_callback(data):
		callback = onNodeStatusUpdate
		if callback is not None:
			callback(data)
	_comms.set_status_callback(status_callback)

# Speech

def say(text):
	_bot.say(text)

# LEDs

_colors = {
	'red': (255,0,0),
	'green': (0,255,0),
	'blue': (0,0,255),
	'white': (255,255,255),
	'blank': (0,0,0),
	'yellow': (255,255,0),
	'pink': (255,0,255)
}

def setEarLED(color, side):
	left = bool(side & 1)
	right = bool(side & 2)
	r, g, b = _colors[color]
	_bot.set_leds(r, g, b, left=left, right=right)

def cancelEarLEDEffect(ledid):
	pass

# Joints

def moveHeadTo(pan, pitch, relative=False, time_to_reach=0.5):
	if relative:
		pan += _bot.joint_states['head_1_joint']
		pitch += _bot.joint_states['head_2_joint']
	_bot.do_trajectory([{'head_1_joint': pan, 'head_2_joint': pitch, 'time_to_reach': time_to_reach}])

def getArmJointPositions(left_arm):
	if left_arm:
		return {key : _bot.joint_states[key] for key in _bot.arm_left[1]}
	else:
		return {key : _bot.joint_states[key] for key in _bot.arm_right[1]}

def moveArmWithJointTrajectory(task):
	def arm_callback(state, result):
		if state == GoalStatus.SUCCEEDED:
			callback = onMoveArmActionSuccess
		else:
			callback = onMoveArmActionFailed
		if callback is not None:
			if 'arm_left_1_joint' in task[0]:
				left = True
			else:
				left = False
			callback(left)
	_bot.do_trajectory(task, arm_callback)

def moveTorsoWithJointTrajectory(task):
	_bot.do_trajectory(task)

def moveTorsoTo(pan, pitch, relative = False, time_to_reach = 2.0):
	if relative:
		pan += _bot.joint_states['torso_1_joint']
		pitch += _bot.joint_states['torso_2_joint']
	task = [{
		'torso_1_joint': pan, 
		'torso_2_joint': pitch, 
		'time_to_reach': time_to_reach
	}]
	_bot.do_trajectory(task)

def directToWeb(url):
	_bot.open_web(url)

def sendMessageToNode(node, message):
	_comms.send_message(node, message)

def playDefaultMotion(motion):
	_bot.play_motion(motion)

# Timer

def addTimer(initial, repeats, interval = None):
	if interval is None:
		interval = initial
	def timer_callback(handle):
		callback = onTimer
		if callback is not None:
			callback(handle)
	return _timer.do_timer(initial, repeats, interval, timer_callback)

# Functionality that is not supported

startDataRecording = None
stopDataRecording = None
onBatteryChargeChange = None

def setLowPowerThreshold(threshold):
	print 'Warning: setLowPowerThreshold not supported in PyREEM.py'


# TODO

onSpeakSuccess = None
onSystemShutdown = None
onTimer = None

def registerPalFaceCallback(facecb):
	print 'registerPalFaceCallback ignored', facecb

def registerTorsoSonarCallback(torsocb):
	print 'registerTorsoSonarCallback ignored', torsocb

def pulseEarLED(colour_one, colour_two, side, period):
	print 'pulseEarLED ignored', colour_one, colour_two, side, period

def startPalFaceEnrollment(name):
	print 'startPalFaceEnrollment ignored', name
def stopPalFaceEnrollment():
	print 'startPalFaceEnrollment ignored'


