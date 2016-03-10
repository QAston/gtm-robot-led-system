LED packages for gtm-robot
-----------

gtm_animation_msgs - .msg files for using gtm-led-control

gtm_led_control - high level api for controlling robot animations for displaying robot status, etc.

gtm_openpixelcontrol_server_fadecandy - fadecandy server controlling LEDs using openpixelcontrol protocol

before using install dependencies:
`rosdep install gtm_openpixelcontrol_server_fadecandy`
`rosdep install gtm_led_control`

test run:
roslaunch gtm_led_control test.launch