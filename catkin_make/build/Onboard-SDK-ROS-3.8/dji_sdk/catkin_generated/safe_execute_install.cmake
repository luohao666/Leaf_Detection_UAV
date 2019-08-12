execute_process(COMMAND "/home/dji/catkin_make/build/Onboard-SDK-ROS-3.8/dji_sdk/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/dji/catkin_make/build/Onboard-SDK-ROS-3.8/dji_sdk/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
