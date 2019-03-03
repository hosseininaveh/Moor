#Joystick Input For Control Of A System

## Overview

This ROS node feeds off of an inbound /joy topic containing joystick buttons and joysticks.  The node then can convert a joystick to the /cmd_vel or twist format outbound topic and can recognize hand controller buttons for other forms of system control.  This node also has some features that are outside of the scope of just a joystick so that the other buttons on our joystick could control other aspects of the bot as if we had a control panel that included a joystick. These other features were done so a simplistic remote front panel would be available without need for WiFi. 

## Dependance on an inbound /joy topic

The system needs to feed this node with the /joy topic from a joystick of choice such as Xbox or Ps3 USB dongle and then the driver software.   Get the joystick drivers as below.

        sudo apt-get install ros-indigo-joystick-drivers

If you plug in a joystick dongle and you do not see /dev/input/js0 then there is no hope that /joy will find the joystick and thus no hope that this package will be able to use topic /joy for input.   Resolve that first.

## Optional Dependance on navigation stack if MoveBase support is desired

The joy_input node has the ability to use  movebase specification to nav targets with slight modifications
If you wish to issue MoveBase commands from the joystick you will need to do these things.  This was something put in that is more along the lines of something we wanted for trade shows so we could run pure bluetooth joystick and not have to get wifi going at a tradeshow.


        sudo apt-get install  ros-indigo-navigation
        cp CMakeLists_with_movebase.txt CMakeLists.txt
        cp package_with_movebase.xml package.xml

Then you will need to enable movebase usage in joy_input.cpp by editing the file and find the #undef USE_MOVEBASE line to change the   undef to a  define.    Now after the catkin_make movebase will be possible per instructions below.

## Configuration of joystick in use

The node was written to support PS3 dualshock and XBox 360 controllers as long as the PS3 controller has an XInput selection to emulate XBox 360 protocol.   We have found that the  'Wireless PS3 Controller To PC USB Adapter [by Mayflash] works if set to Xinput when using PS3 controller.  The XBox dongle we have found to work is the 'PC Wireless Gaming Receiver' which says model 1086 on the tag near the usb plug.

ROS parameters set in the launchfile allow the node to be customized in common ways.   Since the joystick axis moves show up as -1 to +1 for both fwd/reverse and right to left we have included for both the ability to scale the /joy axis value and then cap it to a max if required.  The parameters in joystick.launch are briefly discussed below but are commented in the launch file as well

        enable_joystick           Set to 0 to disable joystick completely (not likely to be needed)
        joystick_deadzone         Defines a deadzone where joystick returns 0. Needed due to cheap joysticks
        cmd_vel_msg_per_sec       Max messages issued per second to throttle too many messages
                                  This defaults to about 4 per second but you can set up to about 10.

        Joystick Specific Parameters. We have a scale applied to /joy value then a max cap applied
        cmd_vel_joy_speed_scale   The joystick fwd/back scale applied to the -1 to 1 /joy values
        cmd_vel_joy_speed_max     The joystick fwd/back max values to /cmd_vel after scaling
        cmd_vel_joy_turn_scale    The joystick right/left scale applied to the -1 to 1 /joy values
        cmd_vel_joy_turn_max      The joystick right/left max values to /cmd_val after scaling

        Button values for speed are here and have single non-changing values
        cmd_vel_btn_speed         The speed used for the forward button (not the joystick)
        cmd_vel_btn_turn_speed    The linear speed used for the forward button (not the joystick)
        cmd_vel_btn_turn_angle    The angular speed used for the right or left button (not the joystick)

        disable_nav_stack         Defaults to 0 as this was a show demo mode. To use this see section above
        target_N_x                Pre-programed move_base x,y,z values for wireless 'goto nav point' use

The joy_input node consumes topic /joy and you may find it of value see the output of the required ros-indigo-joystick-drivers node using this command below.  If you don't see anything on /joy then the joy_input node will have no input.

        rostopic echo /joy

It is not likely that you will have issues but if you have some custom joystick that is recognized and output does come out on /joy but is different than expected then inspect joy topic and find the buttons and joystick axis your controller generates.  If then required, modify the defines at the start of joy_input.cpp to match your controller as required.

## Running the Joystick Input Node

The node is started using roslaunch and you may modify the parameters discussed in the configuration section by use of rosparam list then rosparam set <paramter_name> <value>  at runtime because the parameters are refreshed in the node.

        roslaunch joy_input joystick.launch

The window that starts the node will show the published /cmd_vel values of linear.x and angular.z.
You may also find it of use to look at the /cmd_vel topic to see those values as follows

        rostopic echo /cmd_vel

## Manual operation without a joystick

You can issue messages manually to the /joy topic where first button is 0 (below is a single line!)

rostopic pub /joy sensor_msgs/Joy '{ header: {seq: 10, stamp: {secs: 1431222430, nsecs: 345678}, frame_id: "3"},
   axes: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], buttons: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]}'

## Definition of button mapping to functionality

Here we explain what the buttons do.  Both the XBOX and PS3 have same physical layout for the keys we use but may look a little different so I'll try to explain the buttons for both units.   There are 3 special buttons that either act as enable or function 'shift' buttons that are held when the for colorful keys to the right hand are pushed.

Left Joystick (only when left front top button is held will the joystick operate)
  This joystick controls motion for forward, back and right to left.  Output is only to /cmd_vel topic or the 'twist' messages for classic bot control.  You can move joystick and just click the left top front button or hold button and move stick.

## The 4 multi-function colored keys

These 4 keys each have 3 functions depending on if any 'mode shift keys' are being held.

With no 'mode shift keys held' the 4 buttons will issue a fixed simple set of velocities to the /cmd_vel topic.   Default speeds will be used for these simple move functions.  A couple ros params will set these simple speeds.

    Forward:  The XBox yellow 'Y' (PS3 triangle)
    Stop:     The XBox green  'A' (PS3 X)
    Right:    The XBox red    'B' (PS3 circle)
    Left:     The XBox blue   'X' (PS3 square)


## Direct Hardware Drive   
The XBox 'Start' (PS3 Start) is the Direct Hardware Drive 'mode shift key' held down the above 4 movement commands are issued as if a ROS Arduino bridge is hooked up to the serial port.  The directions will be as above. This was included so you can run the bot without ROS Arduino Bridge.  Some of our robots will not do anything in this mode as without ROS Arduino Bridge the serial goes nowhere. 


## Navigation Target Specification   
Uses XBox 'Back' button (PS3 'Select' button) as a Nav target specification 'mode shift key'.

Support for the MoveBase keys is only possible if the Optional dependency directions have been followed so that navigation is possible.  Please see earlier in this README on modifications to source to allow MoveBase

When held down this causes the 4 colorful keys to issue movebase commands to the 4 configured targets.  Target definitions are given as x,y,w as ROS parameters in the launch file but will be re-read every 5 seconds or so to allow dynamic changing of target nav points as the bot is running.  You must use  rosparam set to change those if already running.


    Target1:  The XBox yellow 'Y' (PS3 triangle)
    Target2:  The XBox red    'B' (PS3 circle)
    Target3:  The XBox blue   'X' (PS3 square)
    Target4:  The XBox green  'A' (PS3 X)

        
