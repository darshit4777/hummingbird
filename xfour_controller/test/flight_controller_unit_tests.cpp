#include <xfour_controller/flight_control.hpp>

// gtest
#include <gtest/gtest.h>

using namespace xfour_controller;

TEST(DesiredRotationMatrix,getOrientation )
{ 

  //ros::init("test_flight_control");
  ros::NodeHandle nh;
  FlightController hummingbird_controller(&nh);
  Eigen::Quaterniond q;
  q = q.UnitRandom();
  ROS_INFO_STREAM("The random quaternion is ..");
  ROS_INFO_STREAM(q.x());
  ROS_INFO_STREAM(q.y());
  ROS_INFO_STREAM(q.z());
  ROS_INFO_STREAM(q.w());
  geometry_msgs::Quaternion qInput;
  qInput.x = q.x();
  qInput.y = q.y();
  qInput.z = q.z();
  qInput.w = q.w();
  qInput.x = 0.0;
  qInput.y = 0.0;
  qInput.z = 0.0;
  qInput.w = 1.0;
  ROS_INFO_STREAM("The input quaternion is ");
  ROS_INFO_STREAM(qInput);
  Eigen::Vector3d orientationVector = hummingbird_controller.GetOrientationVectorFromQuaternion(qInput);
  ROS_INFO_STREAM("The orientation vector is ");
  ROS_INFO_STREAM(orientationVector);
};

