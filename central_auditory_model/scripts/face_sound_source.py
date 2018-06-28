#!/usr/bin/env python
import threading

import rospy
from geometry_msgs.msg import Twist
from central_auditory_model.msg import AngleEstimation


NODE_NAME = 'chase_sound_source'
SUB_TOPIC_NAME = '/central_auditory_model/angle_estimation'

PI = 3.1415926535897

TRIGGER_THRESHOLD = 3
SUPPORTED_ANGLES = [90, 60, 30, 0, 330, 300, 270]

trigger_counter = 0
trigger_index = -1

event = threading.Event()


def ic_cb(data):    
    confidence = float(data.votes[data.angle_index]) / sum(data.votes)
    if not event.is_set():
        global trigger_counter, trigger_index
        
        if trigger_index < 0:
            if confidence > 0.35:
                trigger_index = data.angle_index
        elif trigger_index == data.angle_index:
            if confidence > 0.3:
                if trigger_counter < TRIGGER_THRESHOLD:
                    trigger_counter += 1
                else:
                    event.set()
                    rospy.loginfo('TRIGGERED!!!')
            elif confidence < 0.1:
                if trigger_counter > 0:
                    trigger_counter -= 1
                else:
                    trigger_index = -1
        else:
            if confidence > 0.5:
                trigger_index = data.angle_index
                trigger_counter = 0
            elif confidence > 0.3:
                if trigger_counter > 0:
                    trigger_counter -= 1
                else:
                    trigger_index = -1
                    
        rospy.loginfo('<%.2f, %d> [%d]: %d' % (confidence, data.angle_index, trigger_index, trigger_counter))
        

def do_one_turn(cmd_vel_pub, angle, speed=20., factor=0.9):
    vel_msg = Twist()
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0

    abs_angular_speed = speed * PI / 180.

    if angle > 180:
        relative_angle = (360 - angle) * PI / 180.
        vel_msg.angular.z = -1. * abs_angular_speed
    else:
        relative_angle = angle * PI / 180.
        vel_msg.angular.z = 1. * abs_angular_speed
        
    t0 = rospy.Time.now().to_sec()        
    while factor * abs_angular_speed * (rospy.Time.now().to_sec() - t0) < relative_angle:
        rospy.sleep(0.01)
        cmd_vel_pub.publish(vel_msg)

    vel_msg.angular.z = 0
    cmd_vel_pub.publish(vel_msg)
    rospy.loginfo('chase_sound_source %d done!' % angle)
    rospy.sleep(0.1)


def chase_sound_source():
    rospy.loginfo('chase_sound_source start')
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    try:
        while not rospy.is_shutdown():
            if event.wait(1.0):                
                do_one_turn(cmd_vel_pub, SUPPORTED_ANGLES[trigger_index], speed=40)

                global trigger_counter, trigger_index
                trigger_counter = 0
                trigger_index = -1
                event.clear()
            else:
                pass
    except KeyboardInterrupt:
        pass
    finally:
        cmd_vel_pub.publish(Twist())
        rospy.loginfo('chase_sound_source stopped')
        rospy.sleep(0.5)


if __name__ == '__main__':
    try:
        rospy.init_node(NODE_NAME)
        rospy.Subscriber(SUB_TOPIC_NAME, AngleEstimation, ic_cb)
        rospy.sleep(0.5)

        chase_sound_source()
    except rospy.ROSInterruptException as e:
        rospy.logerr(e)