import os, sys, inspect, thread, time
src_dir = os.path.dirname(inspect.getfile(inspect.currentframe()))
# Windows and Linux
arch_dir = '../lib/x64' if sys.maxsize > 2**32 else '../lib/x86'
# Mac
#arch_dir = os.path.abspath(os.path.join(src_dir, '../lib'))

sys.path.insert(0, os.path.abspath(os.path.join(src_dir, arch_dir)))

import Leap

#def main():
#
#   # Keep this process running until Enter is pressed
#    print "Press Enter to quit..."
#    try:
#        sys.stdin.readline()
#    except KeyboardInterrupt:
#        pass
class SampleListener(Leap.Listener):

    def on_connect(self, controller):
        print "Connected"

    def on_frame(self, controller):
        print "Frame available"
        frame = controller.frame()
        hands = frame.hands.rightmost
        position = hands.palm_position
        velocity = hands.palm_velocity
        direction = hands.direction
        pitch = hands.direction.pitch
        yaw = hands.direction.yaw
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

    # Keep this process running until Enter is pressed
    print "Press Enter to quit..."
    try:
        sys.stdin.readline()
    except KeyboardInterrupt:
        pass
    finally:
        # Remove the sample listener when done
        controller.remove_listener(listener)

if __name__ == "__main__":
    main()
