execute_process(COMMAND "/home/gloria/catkin_ws/build/push_toycar/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/gloria/catkin_ws/build/push_toycar/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
