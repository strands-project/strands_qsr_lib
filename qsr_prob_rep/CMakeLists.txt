cmake_minimum_required(VERSION 2.8.3)
project(qsr_prob_rep)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  rospy
  std_msgs
)

catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate services in the 'srv' folder
add_service_files(
  FILES
  QSRProbRep.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES qsr_hmm_rep
#  CATKIN_DEPENDS ghmm message_generation message_runtime rospy std_msgs std_srvs
#  DEPENDS system_lib
)

###########
## Build ##
###########


#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
install(PROGRAMS
  scripts/qsr_prob_rep_ros_server.py
  scripts/example_hmm_client.py
  scripts/example_pf_client.py
  scripts/create_pf_models.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Testing ##
#############

if (CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS rostest)
  add_rostest(tests/hmm_tester.test)
  add_rostest(tests/pf_tester.test)
endif()
