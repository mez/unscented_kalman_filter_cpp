cmake_minimum_required (VERSION 3.5)
project(UnscentedKF)
add_definitions(-std=c++0x -Wall)

include_directories('libs/Eigen')

###########################################
#  EKF SOURCE CODE
###########################################
set(EKFSOURCES
   utility.cc
   ukf.cc)

###########################################
#  MAIN EKF EXECUTABLE
###########################################
add_executable(UnscentedKF
              ${EKFSOURCES}
              main.cc)

###########################################
#  TESTING SOURCE AND EXECUTABLE
###########################################
set(TESTSOURCES
  ${EKFSOURCES}
  init_tests.cc
  ukf_test.cc
)
add_executable(TestUKF
              ${TESTSOURCES})
