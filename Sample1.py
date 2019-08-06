#!/usr/bin/env python
import rospy
import mavros
from mavros.msg import State
from mavros.srv import CommandBool, SetMode

import os, sys, inspect, thread, time
src_dir = os.path.dirname(inspect.getfile(inspect.currentframe()))
arch_dir = '../lib/x64' if sys.maxsize > 2**32 else '../lib/x86'
sys.path.insert(0, os.path.abspath(os.path.join(src_dir, arch_dir)))
import Leap

current_state = State()
def state_cb(state):
    global current_state
    current_state = state

class SampleListener(Leap.Listener):

    def on_connect(self, controller):
        print("Connected")

    def on_frame(self, controller):
        print "Frame available"
        frame = controller.frame()
        hands = frame.hands.rightmost
        position = hands.palm_position
        velocity = hands.palm_velocity
        direction = hands.direction
        global pitch = hands.direction.pitch
        global yaw = hands.direction.yaw
        roll = hands.palm_normal.roll
        strength = hands.grab_strength
        sphere_center = hands.sphere_center
        #print "Frame id: %d, timestamp: %d, hands: %d, fingers: %d,Pointables : %d" % (
        #     frame.id, frame.timestamp, len(frame.hands), len(frame.fingers),len(frame.pointables))
        #print "position: %d, velocity: %d, direction: %d " % (
        #     position, velocity, direction )

        if(strength > 0.8):
            print("postion_hold_mode")
        else:
            print("stabilized flight mode \n")
            if(roll > 0.5):
                print("roll_right")
            if(roll < -0.5):
                print("roll_left")
            if(yaw > 0.5):
                print("yaw_right")
            if(yaw < -0.5):
                print("yaw_left")
            if(sphere_center[1] > position[1] + 70):
                print("up")
            if(sphere_center[1] < position[1] -30):
                print("down")
            if(sphere_center[2] - 30 > position[2]):
                print("pitch_backward")
            if(sphere_center[2] + 30 < position[2] ):
                print("pitch_forward")




def main():
    # Create a sample listener and controller
    listener = SampleListener()
    controller = Leap.Controller()

    # Have the sample listener receive events from the controller
    controller.add_listener(listener)

    rate = rospy.Rate(20.0)
    #keep on trying to connect if not connected
    while not current_state.connected:
        rate.sleep()

    last_request = rospy.get_rostime()
    while not rospy.is_shutdown():
        now = rospy.get_rostime()
        if current_state.mode != "OFFBOARD" and (now - last_request > rospy.Duration(5.)):
            set_mode_client(base_mode=0, custom_mode="OFFBOARD")
            last_request = now
        else:
            if not current_state.armed and (now - last_request > rospy.Duration(5.)):
               arming_client(True)
               last_request = now

        # older versions of PX4 always return success==True, so better to check Status instead
        if prev_state.armed != current_state.armed:
            rospy.loginfo("Vehicle armed: %r" % current_state.armed)
        if prev_state.mode != current_state.mode:
            rospy.loginfo("Current mode: %s" % current_state.mode)
        prev_state = current_state

        # Update timestamp and publish pose
        pose.header.stamp = rospy.Time.now()
        local_pos_pub.publish(pose)
        rate.sleep()

    try:
        sys.stdin.readline()
    except KeyboardInterrupt:
        pass
    finally:
        # Remove the sample listener when done
        controller.remove_listener(listener)

if __name__ == "__main__":
    main()
