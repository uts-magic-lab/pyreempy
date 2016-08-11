import rospy
import threading

def do_timer(initial, iterations, interval, callback):
	thread = threading.Thread()
	
	def timer_thread():
		rospy.sleep(initial)
		callback(thread)
		if iterations < 0:
			while True:
				rospy.sleep(interval)
				callback(thread)
		elif iterations > 0:
			for i in range(iterations):
				rospy.sleep(interval)
				callback(thread)

	thread = threading.Thread()
	thread.run = timer_thread
	thread.start()
	return thread

