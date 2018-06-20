#!/usr/bin/env python
# license removed for brevity

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from central_auditory_model.msg import AngleEstimation


NODE_NAME = 'chase_sound_source'
SUB_TOPIC_NAME = '/central_auditory_model/ic_stream/angle_estimation'


def movebase_client():

    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "base_link"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = 1.0
    goal.target_pose.pose.orientation.w = 1.0

    rospy.loginfo(goal)

    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        return None
        # rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()


def ic_cb(data):
    confidence = float(data.votes[data.angle_index]) / sum(data.votes)

    if confidence > 0.9:
        rospy.loginfo('Trigger!')
        result = movebase_client()
        if result:
            rospy.loginfo(result)
        rospy.signal_shutdown('signal_shutdown')


if __name__ == '__main__':
    try:
        rospy.init_node(NODE_NAME)
        # rospy.Subscriber(SUB_TOPIC_NAME, AngleEstimation, ic_cb)
        result = movebase_client()
        if result:
            rospy.loginfo("Goal execution done!")
            print result
        else:
            rospy.logwarn("Goal execution failed!")
    except rospy.ROSInterruptException as e:
        rospy.logerr(e)