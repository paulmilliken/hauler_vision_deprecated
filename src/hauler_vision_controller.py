#!/usr/bin/env python
import rospy
from mouse_accumulator.msg import accumulated_mouse_event
from axis_camera.msg import Axis

class HaulerVisionController:
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
        self.axis_msg.autofocus = True
        self.ptz_params = {
                'pan_tilt_sensitivity': 0.05,
                'zoom_sensitivity': 200,
                'pan_left_limit': -90, #-170,
                'pan_right_limit': 90, #170,
                'tilt_down_limit': 30,
                'tilt_up_limit': -45, # camera can go to 90
                'zoom_limit': 8000, #16000, # max is 16000 (32x)
        }
        for key in self.ptz_params.keys():
            self.ptz_params[key] = rospy.get_param(key, self.ptz_params[key])

    def define_ros_variables(self):
        rospy.init_node('hauler_vision_controller')
        self.sub = rospy.Subscriber('accumulated_mouse_event', accumulated_mouse_event, \
                self.mouse_event_callback)
        self.pub = rospy.Publisher('cmd', Axis)

    def mouse_event_callback(self, mouse_event_msg):
        '''responds to mouse_event by publishing to Axis message'''
        rospy.logdebug('A mouse event has occurred')
        if mouse_event_msg.right_displacement!=0:
            self.update_pan(mouse_event_msg)
        if mouse_event_msg.down_displacement!=0:
            self.update_tilt(mouse_event_msg)
        if mouse_event_msg.wheel_up_displacement!=0:
            self.update_zoom(mouse_event_msg)
        # todo: left-click and right-click for brightness
        self.pub.publish(self.axis_msg) # publish PTZ commands
        rospy.logdebug('A ROS message has been published containing PTZ commands')

    def update_pan(self, msg):
        '''update pan parameter in Axis message'''
        self.axis_msg.pan += self.ptz_params['pan_tilt_sensitivity'] * \
                msg.right_displacement
        if self.axis_msg.pan<self.ptz_params['pan_left_limit']:
            self.axis_msg.pan = self.ptz_params['pan_left_limit']
        elif self.axis_msg.pan>self.ptz_params['pan_right_limit']:
            self.axis_msg.pan = self.ptz_params['pan_right_limit']

    def update_tilt(self, msg):
        '''update tilt parameter in Axis message'''
        self.axis_msg.tilt -= self.ptz_params['pan_tilt_sensitivity'] * \
                msg.down_displacement
        if self.axis_msg.tilt>self.ptz_params['tilt_down_limit']:
            self.axis_msg.tilt = self.ptz_params['tilt_down_limit']
        elif self.axis_msg.tilt<self.ptz_params['tilt_up_limit']:
            self.axis_msg.tilt = self.ptz_params['tilt_up_limit']

    def update_zoom(self, msg):
        '''update zoom parameter in Axis message'''
        self.axis_msg.zoom += self.ptz_params['zoom_sensitivity'] * \
                msg.wheel_up_displacement
        if self.axis_msg.zoom<0:
            self.axis_msg.zoom = 0
        elif self.axis_msg.zoom>self.ptz_params['zoom_limit']:
            self.axis_msg.zoom = self.ptz_params['zoom_limit']

def main():
    while not rospy.is_shutdown():
        hauler_vision_controller = HaulerVisionController()
        rospy.spin()

if __name__=='__main__':
    main()
