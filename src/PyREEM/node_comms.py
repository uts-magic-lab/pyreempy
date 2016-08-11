import rospy
from pyride_common_msgs.msg import NodeMessage, NodeStatus

_node_message_publisher = None
_node_status_subscriber = None
_node_status_callback = None

def _node_status_handler(msg):
	callback = _node_status_callback 
	if callback is not None:
		data = {
			'node': msg.node_id,
			'timestamp': msg.header.stamp.to_sec(),
			'priority': msg.priority,
			'message': msg.status_text
		}
		callback(data)

def start():
	global _node_message_publisher, _node_status_subscriber 
	_node_message_publisher = rospy.Publisher('/pyride/node_message', NodeMessage)
	_node_status_subscriber = rospy.Subscriber('/pyride/node_status', NodeStatus, _node_status_handler)

def send_message(node, command, priority = 1):
	msg = NodeMessage()
	msg.node_id = node
	msg.priority = priority
	msg.command = command
	_node_message_publisher.publish(msg)

def set_status_callback(callback):
	global _node_status_callback
	_node_status_callback = callback

