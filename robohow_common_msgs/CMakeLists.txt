cmake_minimum_required(VERSION 2.8.3)
project(robohow_common_msgs)

find_package(catkin REQUIRED COMPONENTS roscpp rospy actionlib_msgs geometry_msgs)

# Generate messages in the 'msg' folder
add_message_files(
  FILES
  MotionPhase.msg
  MotionModel.msg
  GaussianDistribution.msg
  GaussianMixtureModel.msg
)

## Generate actions in the 'action' folder
# add_action_files(
#   DIRECTORY action
#   FILES ConstraintMotion.action
# )


## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs geometry_msgs# Or other packages containing msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES state_transformers
   CATKIN_DEPENDS actionlib_msgs
#  DEPENDS system_lib
)

