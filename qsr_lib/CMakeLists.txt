cmake_minimum_required(VERSION 2.8.3)
project(qsr_lib)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

catkin_python_setup()

add_service_files(
  FILES
  RequestQSRs.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package()

#############
## Install ##
#############

install(PROGRAMS
  scripts/example_extended.py
  scripts/mwe.py
  scripts/qsrlib_ros_server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY share
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if (CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS rostest)
  add_rostest(tests/qsrlib_object_creation_tester.test)
  add_rostest(tests/argd_tester.test)
  add_rostest(tests/argprobd_tester.test)
  add_rostest(tests/cardir_tester.test)
  add_rostest(tests/mos_tester.test)
  add_rostest(tests/mwe_tester.test)
  add_rostest(tests/qtc_tester.test)
  add_rostest(tests/qtcbs_tester.test)
  add_rostest(tests/qtccs_tester.test)
  add_rostest(tests/qtcbcs_tester.test)
  add_rostest(tests/ra_tester.test)
  add_rostest(tests/rcc2_tester.test)
  add_rostest(tests/rcc3_tester.test)
  add_rostest(tests/rcc4_tester.test)
  add_rostest(tests/rcc5_tester.test)
  add_rostest(tests/rcc8_tester.test)
  add_rostest(tests/tpcc_tester.test)
  add_rostest(tests/multiple_tester.test)

endif()
