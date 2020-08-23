#include <gtest/gtest.h>
#include <xfour_controller/flight_control.hpp>
// Run all the tests that were declared with TEST()

int main(int argc, char **argv)
{
  ros::init(argc,argv,"test_flight_control");
  testing::InitGoogleTest(&argc, argv);
  srand((int)time(0));
  return RUN_ALL_TESTS();
};