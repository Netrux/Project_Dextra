#!/usr/bin/env python
import rospy
import mavros
from mavros_msgs.msg import State, OverrideRCIn, RCIn
from mavros_msgs.srv import SetMode

import os, sys, inspect, thread, time
src_dir = os.path.dirname(inspect.getfile(inspect.currentframe()))
arch_dir = '../lib/x64' if sys.maxsize > 2**32 else '../lib/x86'
sys.path.insert(0, os.path.abspath(os.path.join(src_dir, arch_dir)))
import Leap
mavros.set_namespace()
signal = OverrideRCIn()

current_state = State()
def state_cb(state):
    global current_state
    current_state = state
kill=1500
mode=1500
rc_signal = RCIn()
def rc_cb(rcin):
    global rc_signal
    rc_signal = rcin
    global kill
    global mode
    kill = rc_signal.channels[4]
    mode = rc_signal.channels[5]

state_sub = rospy.Subscriber(mavros.get_topic('state'), State, state_cb)
set_mode_client = rospy.ServiceProxy(mavros.get_topic('set_mode'), SetMode)
rc_sub = rospy.Subscriber('mavros/rc/in',RCIn ,10, rc_cb)
rcpub = rospy.Publisher('mavros/rc/override', OverrideRCIn, queue_size=10)
pitch = 0.0
roll = 0.0
yaw = 0.0
strength = 0.0
position = [0,0,0]

class SampleListener(Leap.Listener):

    def on_connect(self, controller):
        print("Connected")

    def on_frame(self, controller):
        print("Frame available")
        frame = controller.frame()
        hands = frame.hands.rightmost
        position = hands.palm_position
        velocity = hands.palm_velocity
        direction = hands.direction
        global pitch
        global yaw
        global roll
        global strength
        global position
        sphere_center = hands.sphere_center
	   # pitch = hands.direction.pitch
        yaw = hands.direction.yaw
        roll = hands.palm_normal.roll
        strength = hands.grab_strength
        sphere_center = hands.sphere_center
        #print "Frame id: %d, timestamp: %d, hands: %d, fingers: %d,Pointables : %d" % (
        #     frame.id, frame.timestamp, len(frame.hands), len(frame.fingers),len(frame.pointables))
        #print "position: %d, velocity: %d, direction: %d " % (
        #     position, velocity, direction )

        if hands.is_valid:
            if(strength > 0.8):
                print("postion_hold_mode")
            else:
                print("stabilized flight mode \n")
                if(roll > 0.5 and roll < 2):
                    print("roll_right")
                if(roll < -0.5 and roll > -2):
                    print("roll_left")
                if(yaw > 0.5 and yaw < 2):
                    print("yaw_right")
                if(yaw < -0.5 and yaw > -2):
                    print("yaw_left")
                if(position[1] > 400):
                    print("up")
                if(position[1] < 130 and position[1] > 10):
                    print("down")
                if(position[2] > 80):
                    print("pitch_backward")
                if(position[2] < -80):
                    print("pitch_forward")

        else :
            strength = 1




def main():
    # Create a sample listener and controller
    rospy.init_node('offb_node', anonymous=True)
    listener = SampleListener()
    controller = Leap.Controller()
    print("pass 1")

    # Have the sample listener receive events from the controller
    controller.add_listener(listener)

    rate = rospy.Rate(70.0)
    # Keep on trying to connect if not connected
    while not current_state.connected:
        rate.sleep()
    print("pass 2")
    last_request = rospy.get_rostime()
    prev_state = current_state
    while not rospy.is_shutdown():
        signal.channels = [1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500]
        print("rospy is active")
        if current_state.mode != "POSCTL" and strength>0.8 :
            set_mode_client(base_mode=0, custom_mode="POSCTL")
            print("asked for postion_hold_mode")
        else:
            if(roll > 0.5 and roll < 2):
                signal.channels[0] = 1300
                print("roll_left published")
            elif(roll < -0.5 and roll > -2):
                signal.channels[0] = 1700
                print("roll_right published")
            if(yaw > 0.5 and yaw < 2):
                signal.channels[3] = 1700
                print("yaw_right published")
            elif(yaw < -0.5 and yaw > -2):
                signal.channels[3] = 1300
                print("yaw_left published")
            elif(position[1] > 400):
                signal.channels[2] = 1700
            elif(position[1] < 120 and position[1] > 10):
                signal.channels[2] = 1300
            elif(position[2] > 80):
                signal.channels[1] = 1300
            elif(position[2] < -80):
                signal.channels[1] = 1700

        rcpub.publish(signal)
        print("signal published")


        # older versions of PX4 always return success==True, so better to check Status instead
        if prev_state.armed != current_state.armed:
            rospy.loginfo("Vehicle armed: %r" % current_state.armed)
        if prev_state.mode != current_state.mode:
            rospy.loginfo("Current mode: %s" % current_state.mode)
        prev_state = current_state
        # Update timestamp and publish pose
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
