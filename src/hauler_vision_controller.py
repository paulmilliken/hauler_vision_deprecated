#!/usr/bin/env python
import rospy
from mouse_driver.msg import mouse_event
from axis_camera.msg import Axis

class Hauler_vision_controller:
    '''This type of ROS node subscribes to a ROS topic of the type mouse_event
    and it will monitor mouse events in /dev/input/event*.  A desired camera
    position is calculated from the mouse motion and published to a topic of
    type Axis.  Thus, the node links the mouse to the PTZ functions of an Axis
    camera.'''

    def __init__(self):
        self.define_variables()
        self.define_ros_variables()

    def define_variables(self):
        self.axis_msg = Axis() # instantiate Axis message
        default_values = {
                'pan_tilt_sensitivity': 0.1,
                'zoom_sensitivity': 200}
        self.pan_tilt_sensitivity = rospy.get_param('pan_tilt_sensitivity', \
                default_values['pan_tilt_sensitivity'])
        self.zoom_sensitivity = rospy.get_param('zoom_sensitivity', \
                default_values['zoom_sensitivity'])
        
    def define_ros_variables(self):
        rospy.init_node('hauler_vision_controller')
        self.sub = rospy.Subscriber('mouse_event', mouse_event, \
                self.mouse_event_callback)
        self.pub = rospy.Publisher('cmd', Axis)

    def mouse_event_callback(self, mouse_event_msg):
        '''responds to mouse_event by publishing to Axis message'''
        rospy.logdebug('A mouse event has occurred')
        if mouse_event_msg.right_displacement!=0:
            self.axis_msg.pan += self.pan_tilt_sensitivity * \
                                        mouse_event_msg.right_displacement
        elif mouse_event_msg.down_displacement!=0:
            self.axis_msg.tilt += -self.pan_tilt_sensitivity * \
                                        mouse_event_msg.down_displacement
        elif mouse_event_msg.wheel_up_displacement!=0:
            self.axis_msg.zoom += self.zoom_sensitivity * \
                                        mouse_event_msg.wheel_up_displacement
        # todo: left-click and right-click for brightness
        self.pub.publish(self.axis_msg) # publish PTZ commands
        rospy.logdebug('A ROS message has been published containing PTZ commands')
    
    def start(self):
        while not rospy.is_shutdown():
            rospy.spin() 

def main():
    try:
        hauler_vision_controller = Hauler_vision_controller()
        hauler_vision_controller.start()
    except rospy.ROSInterruptException:
        rospy.logwarn('ROSInterruptionException has occurred')
        pass

if __name__=='__main__':
    main()
