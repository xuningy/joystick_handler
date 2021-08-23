# Joystick Handler
### Author: Xuning Yang
A package that reads the raw `/joy` message, and filters it with a deadband and timing.

* If the 1-euro filter is enabled, then it is applied to the raw joystick input
* The joy message is pulled at 10Hz in order to eliminate inputs that are too close invalue.

* Requires package `joystick_ui` which contains a `JoyMapper` function, which maps certain keys to certain values in the `/joy` message. This external package can be removed and replaced with default values in initialization.

* Outputs a clean message for the teleoperation stack with the message `JoystickValues`.
