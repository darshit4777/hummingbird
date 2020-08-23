#include "ros/ros.h"
#include <Eigen/Dense>
#include "xfour_controller/flight_control.hpp"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Quaternion.h"
#include "algorithm"
#include "mav_msgs/Actuators.h"
#include "xfour_controller/YawVelocity.h"
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

    m_velocityCommanded.setZero();
    m_velocityCommandedPrevious.setZero();
    m_commandedYaw = 0.0;
    m_commandedYawPrevious = 0.0;
    
    // Initializing all tunable parameters 
    m_tuningParams.gamma = 1.0;
    m_tuningParams.k_eta = 1.0;
    m_tuningParams.k_thrust = 1.0;
    m_tuningParams.sigma = 1.0;
    m_loopTime = 0.1;
    m_velocityErrorLimit[0] = 2.0;
    m_velocityErrorLimit[1] = 2.0;
    m_velocityErrorLimit[2] = 2.0;



    // Initializing physical constants
    m_mass = 1.0; 
    m_g = 9.8134;

    _positionSubscriber = _nh.subscribe("/hummingbird/ground_truth/pose",10,&FlightController::PositionCallback,this);
    _inertialSubscriber = _nh.subscribe("/hummingbird/imu",10,&FlightController::InertialCallback,this);
    _velocitySubscriber = _nh.subscribe("/hummingbird/ground_truth/odometry",10,&FlightController::VelocityCallback,this);
    _commandSubscriber = _nh.subscribe("/controller_command",10,&FlightController::CommandCallback,this);

    _motorCommandPublisher = _nh.advertise<mav_msgs::Actuators>("/hummingbird/command/motors",10);

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
    // Transforming the velocity commands into the control coordinate system before calculating thrust
    Eigen::Vector3d velocityCommanded, velocityCommandedPrevious;
    velocityCommanded = TransformMeasuredToControlCoordinates(m_velocityCommanded);
    ROS_INFO("The transformed velocity is ");
    ROS_INFO_STREAM(velocityCommanded);
    velocityCommandedPrevious = TransformMeasuredToControlCoordinates(m_velocityCommandedPrevious);

    velocityCommanded_derivative = (velocityCommanded -  velocityCommandedPrevious)/m_loopTime;
    ROS_INFO("The velocity derivative is ");
    ROS_INFO_STREAM(velocityCommanded_derivative);
    velocityCommanded_derivative[2] = velocityCommanded_derivative[2] - m_g;

    Eigen::Vector3d saturatedEpsilon;
    // Transforming velocity to the control coordinates before calculating Epsilon
    Eigen::Vector3d velocity = TransformMeasuredToControlCoordinates(m_velocity);
    saturatedEpsilon = CaclulateSaturationEpsilon(velocity,velocityCommanded);
    
    
    m_thrustVector = -m_mass*velocityCommanded_derivative + m_tuningParams.k_thrust * saturatedEpsilon;
    
    // TODO : The sign of the thrust vector doesnt make sense right now - it should have a negative Z component.
    // Let's leave it be for now - we can figure it out later. Therefore a transformation back to measured coordinates,
    // is not needed right now. The sign of thrust may be accounted for elsewhere.
    //thrustVector = TransformControlToMeasuredCoordinates(thrustVector);
    
    ROS_INFO_STREAM("The thrust vector is ..");
    ROS_INFO_STREAM(m_thrustVector);    
    m_thrust = std::sqrt(m_thrustVector.squaredNorm());
    
    return; 
};

Eigen::Vector3d FlightController::CaclulateSaturationEpsilon(Eigen::Vector3d velocity, Eigen::Vector3d velocityCommanded){
    /**
     * This function serves as the saturation function for epsilon.
     * Epsilon is calculated as, epsilon = velocity - velocityCommanded
     */

    Eigen::Vector3d epsilon;
    epsilon.setZero(); 
    epsilon = (velocity - velocityCommanded);

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
    ROS_INFO("The value of epsilon is ");
    ROS_INFO_STREAM(epsilon);
    return epsilon;
        
}

void FlightController::PositionCallback(const geometry_msgs::Pose::ConstPtr& position){
    // Sensor Callback from ground truth or position sensors
    m_position = *position;
    return;
};

void FlightController::InertialCallback(const sensor_msgs::Imu::ConstPtr& imu){
    // Sensor Callback from the IMU sensor
    sensor_msgs::Imu imuMessage = *imu;
    m_angularVelocityPrevious = m_angularVelocity;
    m_angularVelocity[0] = imuMessage.angular_velocity.x;
    m_angularVelocity[1] = imuMessage.angular_velocity.y;
    m_angularVelocity[2] = imuMessage.angular_velocity.z;
    m_acceleration[0] = imuMessage.linear_acceleration.x;
    m_acceleration[1] = imuMessage.linear_acceleration.y;
    m_acceleration[2] = imuMessage.linear_acceleration.z;
    m_orientationQuaternionPrevious = m_orientationQuaternion;
    m_orientationQuaternion = imuMessage.orientation;

    Eigen::Quaterniond q;
    q.x() = m_orientationQuaternion.x;
    q.y() = m_orientationQuaternion.y;
    q.z() = m_orientationQuaternion.z;
    q.w() = m_orientationQuaternion.w;

    m_currentRotationMatrix = q.normalized().toRotationMatrix();
    return;
};

void FlightController::VelocityCallback(const nav_msgs::Odometry::ConstPtr& measuredOdometry){
    // Sensor Callback for velocity

    nav_msgs::Odometry odometryMessage = *measuredOdometry;
    m_velocityPrevious = m_velocity;
    m_velocity[0] = odometryMessage.twist.twist.linear.x;
    m_velocity[1] = odometryMessage.twist.twist.linear.y;
    m_velocity[2] = odometryMessage.twist.twist.linear.z;
    
    return;
};

void FlightController::CommandCallback(const xfour_controller::YawVelocity::ConstPtr& command){
    // Callback for commands to robot

    m_velocityCommandedPrevious = m_velocityCommanded;
    m_velocityCommanded[0] = command->velocity_x;
    m_velocityCommanded[1] = command->velocity_y;
    m_velocityCommanded[2] = command->velocity_z;
    m_commandedYawPrevious = m_commandedYaw;
    m_commandedYaw = command->yaw;
    return;
};

Eigen::Vector3d FlightController::TransformMeasuredToControlCoordinates(Eigen::Vector3d measuredCoordinates){
    /**
     * The coordinate frame in which the measurements are made is a general Z - up coordinated system.
     * The coordinate frame for calculating control inputs is a Z - down coordinate system. This function
     * rotates the incoming vector by 180 degrees about the X axis
    */

    // Using Eigen Axis Angle transform
    Eigen::Vector3d xAxis;
    
    xAxis << 1.0 , 0.0 , 0.0;       // Creating the X Axis
    double rotationAngle = M_PI;    // Rotation by pi
    Eigen::Transform<double,3,2> measuredToControl;
    measuredToControl = Eigen::AngleAxis<double>(rotationAngle,xAxis);                          // Creating the Transform
    Eigen::Vector3d controlCoordinates = measuredToControl * measuredCoordinates;       // Multiplying transform to get control Coordinates
    
    return controlCoordinates;  
};

Eigen::Vector3d FlightController::TransformControlToMeasuredCoordinates(Eigen::Vector3d controlCoordinates){
    /**
     * The coordinate frame in which the measurements are made is a general Z - up coordinated system.
     * The coordinate frame for calculating control inputs is a Z - down coordinate system. This function
     * rotates the incoming vector by 180 degrees about the X axis
    */

    // Using Eigen Axis Angle transform
    Eigen::Vector3d xAxis;
    
    xAxis << 1.0 , 0.0 , 0.0;       // Creating the X Axis
    double rotationAngle = M_PI;    // Rotation by pi
    Eigen::Transform<double,3,2> controlToMeasured;
    controlToMeasured = Eigen::AngleAxis<double>(rotationAngle,xAxis);              // Creating the Transform
    Eigen::Vector3d measuredCoordinates = controlToMeasured * controlCoordinates;   // Multiplying transform to get control Coordinates
    
    return measuredCoordinates;  
};

Eigen::Vector3d FlightController::GetAxisForDesiredRotation(Eigen::Vector3d firstVector, Eigen::Vector3d secondVector){
    /** 
     * This function will calculate the axis of rotation between two vectors by using a vector
     * cross product.
    */

    Eigen::Vector3d rotationAxis;
    rotationAxis = firstVector.cross(secondVector);
    return rotationAxis;
};

double FlightController::GetAngleForDesiredRotation(Eigen::Vector3d firstVector, Eigen::Vector3d secondVector){
    /**
     * This function will calculate the angle between two vectors in 3d space, about an axis 
     * perpendicular to both the vectors. It uses the vector dot product for the same.
     */
    
    double rotationAngle;
    rotationAngle = firstVector.dot(secondVector);
    return rotationAngle;
};

Eigen::Matrix3d FlightController::GetDesiredRotationMatrix(){
    /**
     * This function examines the calculated thrust vector and creates a desired rotation matrix
     * The rotation matrix is defined as a rotation of the body frame with respect to the inertial
     * frame
     */
    Eigen::Matrix3d desiredRotationMatrix;
    Eigen::Vector3d normalizedThrustVector, orientationVector;
    normalizedThrustVector = m_thrustVector.normalized();
    orientationVector = GetOrientationVectorFromQuaternion(m_orientationQuaternion);
    
    /**
     * In the generation of the desired rotation matrix we, first generate the axis of rotation and
     * the angle of rotation. In doing this we follow the convention of using the first vector as 
     * the initial orientation vector in the world frame and the thrust vector as the second vector.
     */
    Eigen::Vector3d initialOrientation;
    initialOrientation << 0.0 , 0.0 , 1.0;
    double rotationAngle;
    Eigen::Vector3d rotationAxis;
    rotationAngle = GetAngleForDesiredRotation(initialOrientation,normalizedThrustVector);
    rotationAxis = GetAxisForDesiredRotation(initialOrientation,normalizedThrustVector);
    
    // TODO : It is still not clear if the usage of the thrust vector with its current sign produces a 
    // correct desired rotation matrix. We would have to fix that at some point.
    desiredRotationMatrix = Eigen::AngleAxis<double>(rotationAngle,rotationAxis);
    //ROS_INFO_STREAM(desiredRotationMatrix);

    return m_desiredRotationMatrix;
};
Eigen::Matrix3d FlightController::GetErrorRotationMatrix(){
    /**
     * This function generates an error rotation matrix Rtilda. 
     * Rtilda = R * Rdtranspose
     */

    m_errorRotationMatrix = m_currentRotationMatrix * m_desiredRotationMatrix.transpose();

    return m_errorRotationMatrix;

};

Eigen::Vector3d FlightController::GetOrientationVectorFromQuaternion(geometry_msgs::Quaternion orientationQuaternion){
    /**
     * This function returns an orientation vector given an orientation quaternion
     */
    Eigen::Quaterniond q;
    q.x() = orientationQuaternion.x;
    q.y() = orientationQuaternion.y;
    q.z() = orientationQuaternion.z;
    q.w() = orientationQuaternion.w;
    
    Eigen::Vector3d initialOrientation;
    initialOrientation << 0.0, 0.0, 1.0;
    Eigen::Vector3d directionCosines;

    directionCosines = q * initialOrientation;
    return directionCosines;
};

/** TODO 
 * 1. Convert the error rotation matrix into an error quaternion.
 * 2. Write methods for calculating angular velocity derivative and other quantities
 * 3. Write a method for making skew symmetric matrices out of vectors.
 * 4. Write a method for calculating Torque
 * 5. Write a method for calculating motor RPMs out of thrust and torque.
 * 
*/ 


int main(int argc, char **argv){
    // Initialize the node 
    
    ros::init(argc,argv,"flight_control");
    ros::NodeHandle nh;
    FlightController HummingBirdController(&nh);
    while(ros::ok()){
        HummingBirdController.CalculateThrust();
        ROS_INFO("The commanded thrust is %f",HummingBirdController.m_thrust);
        ros::spinOnce();    
    };
    
    

}