#include "ros/ros.h"
#include <Eigen/Dense>
#include "geometry_msgs/Pose.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Quaternion.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "xfour_controller/YawVelocity.h"

class FlightController
{
    /**
        This class will function as a lower level controller which accepts
        velocity and yaw commands and provides methods to translate the same
        into motor commands.This class like other flight controllers will
        inherit an interface to allow modularity of a controller
    */

public:
    // Listing all the variables and datatypes

    // State Variables of the robot 
    Eigen::Vector3d m_velocity;                     // Velocity of the robot in 3d space
    Eigen::Vector3d m_velocityPrevious;             // Velocity of the robot in the previous time step
    Eigen::Vector3d m_acceleration;                 // Acceleartion of the the robot in 3d space
    geometry_msgs::Pose m_position;                     // Position of the robot in 3d space
    struct orientation{
        double roll;
        double pitch;
        double yaw;
    } m_robotOrientation;
    geometry_msgs::Quaternion m_orientationQuaternion;
    geometry_msgs::Quaternion m_orientationQuaternionPrevious;
    orientation m_robotOrientationPrevious;           // Previous yaw, pitch and roll
    Eigen::Vector3d m_angularVelocity;              // Angular velocity of the robot 
    Eigen::Vector3d m_angularVelocityPrevious;      // Angular velocity in the previous time step
    Eigen::Vector3d m_angularAcceleration;          // Angular acceleration of the robot
    double m_torqueGyroscopic;  // Gyroscopic torque
    
    // Reference or Commanded Variables
    Eigen::Vector3d m_velocityCommanded;            // Commanded or reference velocity of the robot
    Eigen::Vector3d m_velocityCommandedPrevious;    // Commanded velocity of the robot in the previous time step
    double m_commandedYaw;
    double m_commandedYawPrevious;
    Eigen::Matrix3d m_desiredRotationMatrix;        // The desired rotaion matrix computed for convergence of rotation dynamics

    // Controller Outputs    
    double m_thrust;                                // Thrust output generated by the controller
    Eigen::Vector3d m_thrustVector;                 // The vector representation of Thrust
    Eigen::Vector3d m_torque;                       // Torque outputs generated by controller
    struct motor_commands{
        double motor1;
        double motor2;
        double motor3;
        double motor4; 
    } m_motorCommands;
    
    // Physical Constants
    double m_motorConstant;                         // Motor constant that relates propeller thrust to motor rpm
    double m_mass;                                  // Mass of the robot
    Eigen::Vector3d m_rotationalInertia;            // Rotational inerta of the robot. Ixx, Iyy and Izz
    double m_g;     // Acceleration due to gravity

    // Controller Related Variables
    struct tuning_constants{
        double k_eta;
        double sigma;
        double k_thrust;
        double gamma;
    } m_tuningParams;       // Constants to tune the controller       
    double m_loopTime;      // Time dt of control loop
    Eigen::Vector3d m_velocityErrorLimit;
         
private:
    // ROS subscribers and publishers
    ros::NodeHandle _nh;
    ros::Subscriber _positionSubscriber;
    ros::Subscriber _inertialSubscriber;
    ros::Subscriber _velocitySubscriber;
    ros::Subscriber _orientationSubscriber;
    ros::Subscriber _commandSubscriber;
    ros::Publisher _motorCommandPublisher;

// Listing all methods
public:
/** Class constructor */
FlightController(ros::NodeHandle* nodehandle);

void PositionCallback(const geometry_msgs::Pose::ConstPtr& position);
void InertialCallback(const sensor_msgs::Imu::ConstPtr& imu);
void VelocityCallback(const nav_msgs::Odometry::ConstPtr& measuredOdometry);
void CommandCallback(const xfour_controller::YawVelocity::ConstPtr& commandMessage);
void CalculateThrust();
Eigen::Matrix3d GetDesiredRotationMatrix();
Eigen::Vector3d CaclulateSaturationEpsilon(Eigen::Vector3d velocity, Eigen::Vector3d velocityCommanded);
Eigen::Vector3d TransformMeasuredToControlCoordinates(Eigen::Vector3d measuredCoordinates);
Eigen::Vector3d TransformControlToMeasuredCoordinates(Eigen::Vector3d controlCoordinates);
Eigen::Vector3d GetAxisForDesiredRotation(Eigen::Vector3d firstVector, Eigen::Vector3d secondVector);
Eigen::Vector3d GetOrientationVectorFromQuaternion(geometry_msgs::Quaternion orientationQuaternion);
double GetAngleForDesiredRotation(Eigen::Vector3d firstVector, Eigen::Vector3d secondVector);

};

