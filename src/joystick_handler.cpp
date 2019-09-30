#include <joystick_handler/JoystickHandler.h>

int main (int argc, char **argv)
{
  ros::init(argc, argv, "joystick_handler");
  ros::NodeHandle node("~");

  planner::JoystickHandler jh;

  if (!jh.initialize(node))
  {
    ROS_ERROR("%s: failed to initialize joystick handler",
        ros::this_node::getName().c_str());
    return EXIT_FAILURE;
  }

  ros::spin();

  return EXIT_SUCCESS;
}
