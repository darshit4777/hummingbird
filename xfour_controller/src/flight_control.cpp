#include "ros/ros.h"
#include <Eigen/Dense>
#include "flight_control.hpp"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Quaternion.h"
#include "algorithm"
FlightController::FlightController(ros::NodeHandle* nodehandle)
{
    
    // Initializing all state variables to zero
    m_velocity.setZero();
    m_velocityPrevious.setZero();


    m_robotOrientation.roll = 0.0;
    m_robotOrientation.pitch = 0.0;
    m_robotOrientation.yaw = 0.0;
    m_robotOrientationPrevious.roll = 0.0;
    m_robotOrientationPrevious.pitch = 0.0;
    m_robotOrientationPrevious.yaw = 0.0;

    m_angularVelocity.setZero();
    m_angularVelocityPrevious.setZero();
    m_torqueGyroscopic = 0.0;

    // Initializing all commanded variables to zero
    m_motorCommands.motor1 = 0.0;
    m_motorCommands.motor2 = 0.0;
    m_motorCommands.motor3 = 0.0;
    m_motorCommands.motor4 = 0.0;

    m_thrust = 0.0;
    m_torque.setZero();
    
    // Initializing all tunable parameters 
    m_tuningParams.gamma = 1.0;
    m_tuningParams.k_eta = 1.0;
    m_tuningParams.k_thrust = 1.0;
    m_tuningParams.sigma = 1.0;
    m_loopTime = 0.1;

    // Initializing physical constants
    m_mass = 1.0; 
    m_g = 9.8134;

    _positionSubscriber = _nh.subscribe("/hummingbird/ground_truth/pose",10,&FlightController::m_positionCallback,this);
    _inertialSubscriber = _nh.subscribe("/hummingbird/imu",10,&FlightController::m_inertialCallback,this);
    _velocitySubscriber = _nh.subscribe("/commanded_velocity",10,&FlightController::m_velocityCallback,this);
    _orientationSubscriber = _nh.subscribe("/commanded_orientation",10,&FlightController::m_orientationCallback,this);

    _motorCommandPublisher = _nh.advertise('/hummingbird/command/motors',10);

    return;
};

void FlightController::CalculateThrust(){
    /** 
     * Thrust is calculate by the following equation
     * Thrust = || -mass * (velocityCommanded_dot - g . e3) + k1 * sat (epsilon) ||
     * Here the epsilon = velocity - velocityCommanded
     * The saturation function for epsilon is defined as a separate method
     */

    Eigen::Vector3d velocityCommanded_derivative;
    velocityCommanded_derivative = (m_velocityCommanded -  m_velocityCommandedPrevious)/m_loopTime;
    
    velocityCommanded_derivative[2] = velocityCommanded_derivative[2] - m_g;

    Eigen::Vector3d saturatedEpsilon;
    saturatedEpsilon = CaclulateSaturationEpsilon(m_velocity,m_velocityCommanded);
    saturatedEpsilon = m_tuningParams.k_thrust * saturatedEpsilon;
    
    Eigen::Vector3d thrustVector;
    thrustVector = -m_mass*velocityCommanded_derivative + saturatedEpsilon;
    
    m_thrust = thrustVector.squaredNorm();
    return; 
};

Eigen::Vector3d FlightController::CaclulateSaturationEpsilon(Eigen::Vector3d velocity, Eigen::Vector3d velocityCommanded){
    /**
     * This function serves as the saturation function for epsilon.
     * Epsilon is calculated as, epsilon = velocity - velocityCommanded
     */

    Eigen::Vector3d epsilon;
    epsilon.setZero();
    epsilon = velocity - velocityCommanded;

    // Saturation
    if (epsilon[0] < 0){
        epsilon[0] = std::max(epsilon[0],-m_velocityErrorLimit[0]);
    }
    else
    {
        epsilon[0] = std::min(epsilon[0],m_velocityErrorLimit[0]);
    };

    if (epsilon[1] < 0){
        epsilon[1] = std::max(epsilon[1],-m_velocityErrorLimit[1]);
    }
    else
    {
        epsilon[1] = std::min(epsilon[1],m_velocityErrorLimit[1]);
    };
    
    if (epsilon[2] < 0){
        epsilon[2] = std::max(epsilon[2],-m_velocityErrorLimit[2]);
    }
    else
    {
        epsilon[2] = std::min(epsilon[2],m_velocityErrorLimit[2]);
    };
    
    return epsilon;
        
}

void FlightController::PositionCallback(const geometry_msgs::PoseStamped* position){
    m_position = *position;
    return;
};

void FlightController::InertialCallback(const sensor_msgs::Imu* imu){
    sensor_msgs::Imu imuMessage = *imu;
    m_angularVelocity[0] = imuMessage.angular_velocity.x;
    m_angularVelocity[1] = imuMessage.angular_velocity.y;
    m_angularVelocity[2] = imuMessage.angular_velocity.z;
    m_acceleration[0] = imuMessage.linear_acceleration.x;
    m_acceleration[1] = imuMessage.linear_acceleration.y;
    m_acceleration[2] = imuMessage.linear_acceleration.z;

    return;
};

void FlightController::VelocityCallback(const geometry_msgs::Twist* commandedTwist){
    
    geometry_msgs::Twist twistMessage = *commandedTwist;
    m_velocityCommandedPrevious = m_velocityCommanded;
    m_velocityCommanded[0] = twistMessage.linear.x;
    m_velocityCommanded[1] = twistMessage.linear.y;
    m_velocityCommanded[2] = twistMessage.linear.z;
    

}



int main(int argc, char **argv){
    // Initialize the node 
    ros::init(argc,argv,"flight_control");

    ros::spin();

}