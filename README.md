# Joystick Handler
A package that reads the raw `/joy` message, and filters it with a deadband and timing.

* If the 1-euro filter is enabled, then it is applied to the raw joystick input
* The joy message is pulled at 10Hz in order to eliminate inputs that are too close invalue.
