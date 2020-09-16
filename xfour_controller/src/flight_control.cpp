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
    m_torqueVector.setZero();

    m_velocityCommanded.setZero();
    m_velocityCommandedPrevious.setZero();
    m_commandedYaw = 0.0;
    m_commandedYawPrevious = 0.0;
    
    // Initializing all tunable parameters 
    m_tuningParams.gamma = 20.0;
    m_tuningParams.k_eta = 5.0;
    m_tuningParams.k_thrust = 1.0;
    m_tuningParams.k_sigma = 5.0;
    m_loopTime = 0.1;
    m_velocityErrorLimit[0] = 0.1;
    m_velocityErrorLimit[1] = 0.1;
    m_velocityErrorLimit[2] = 0.2;



    // Initializing physical constants - these are set for the hummingbird quadcopter
    m_mass = 0.68;
    m_rotationalInertia << 0.007 , 0.0 , 0.0,
                           0.0 , 0.007 , 0.0,
                           0.0 , 0.0 , 0.012; 
    m_g = 9.8134;
    // Data for a 10x6 Propeller at 1000 rpm, 0 mph. 
    m_motorThrustConstant = 0.000009008; //0.000010008;
    m_propellerOffset = 0.17;
    m_motorTorqueConstant = 0.000000261;
    
    _positionSubscriber = _nh.subscribe("/hummingbird/ground_truth/pose",10,&FlightController::PositionCallback,this);
    auto positionCheck = ros::topic::waitForMessage<geometry_msgs::Pose>("/hummingbird/ground_truth/pose",this->_nh);
    
    _inertialSubscriber = _nh.subscribe("/hummingbird/imu",10,&FlightController::InertialCallback,this);
    auto intertialCheck = ros::topic::waitForMessage<sensor_msgs::Imu>("/hummingbird/imu",this->_nh);
    
    _velocitySubscriber = _nh.subscribe("/hummingbird/ground_truth/odometry",10,&FlightController::VelocityCallback,this);
    auto odometryCheck = ros::topic::waitForMessage<nav_msgs::Odometry>("/hummingbird/ground_truth/odometry",this->_nh);


    _commandSubscriber = _nh.subscribe("/controller_command",10,&FlightController::CommandCallback,this);

    motorCommandPublisher = _nh.advertise<mav_msgs::Actuators>("/hummingbird/command/motor_speed",10);
    debugErrorPublisher = _nh.advertise<geometry_msgs::PoseStamped>("/debug/error",10);
    debugThrustVectorPublisher = _nh.advertise<geometry_msgs::PoseStamped>("/debug/thrust_vector",10);
    debugTorqueVectorPublisher = _nh.advertise<geometry_msgs::PoseStamped>("/debug/torque_vector",10);
    debugDesiredRotationPublisher = _nh.advertise<geometry_msgs::PoseStamped>("/debug/desired_rotation",10);
    debugDesiredAngularVelocityPublisher = _nh.advertise<geometry_msgs::PoseStamped>("/debug/desired_angular_velocity",10);

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
    Eigen::Vector3d velocityCommanded, velocityCommandedPrevious, velocityError;
    velocityCommanded = TransformMeasuredToControlCoordinates(m_velocityCommanded);
    velocityCommandedPrevious = TransformMeasuredToControlCoordinates(m_velocity);
    velocityError = velocityCommanded - velocityCommandedPrevious;

    velocityError[0] = std::max(velocityError[0],-1.0);
    velocityError[0] = std::min(velocityError[0],1.0);

    velocityError[1] = std::max(velocityError[1],-1.0);
    velocityError[1] = std::min(velocityError[1],1.0);
    
    velocityError[2] = std::max(velocityError[2],-1.0);
    velocityError[2] = std::min(velocityError[2],1.0);
    
    
    
    if ( (velocityError[0] < 0.01 ) && (velocityError[0] > -0.01) ){
        velocityError[0] = 0.0;
    };

    if ( (velocityError[1] < 0.01 ) && (velocityError[1] > -0.01) ){
        velocityError[1] = 0.0;
    };

    if ( (velocityError[2] < 0.01 ) && (velocityError[2] > -0.01) ){
        velocityError[2] = 0.0;
    };

    


    //ROS_INFO_STREAM("Velocity Error");  
    //ROS_INFO_STREAM(velocityError);
    velocityCommanded_derivative = (velocityError)/m_loopTime;
    velocityCommanded_derivative[2] = velocityCommanded_derivative[2] - m_g;
    


    Eigen::Vector3d saturatedEpsilon;
    // Transforming velocity to the control coordinates before calculating Epsilon
    saturatedEpsilon = CaclulateSaturationEpsilon(velocityCommandedPrevious,velocityCommanded);
    //ROS_INFO_STREAM("Velocity Derivative Term");
    //ROS_INFO_STREAM(-m_mass*velocityCommanded_derivative);

    //ROS_INFO_STREAM("Saturated Control Term");
    //ROS_INFO_STREAM(m_tuningParams.k_thrust * saturatedEpsilon);
    m_thrustVector = -m_mass*velocityCommanded_derivative + m_tuningParams.k_thrust * saturatedEpsilon;
        
    // TODO : The sign of the thrust vector doesnt make sense right now - it should have a negative Z component.
    // Let's leave it be for now - we can figure it out later. Therefore a transformation back to measured coordinates,
    // is not needed right now. The sign of thrust may be accounted for elsewhere.
    //thrustVector = TransformControlToMeasuredCoordinates(thrustVector);
    
    //ROS_INFO_STREAM("The thrust vector is ..");
    //ROS_INFO_STREAM(m_thrustVector);    
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
    //ROS_INFO("The value of saturation epsilon is ");
    //ROS_INFO_STREAM(epsilon);
    return epsilon;
        
};


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
    geometry_msgs::Quaternion qMeasured;
    qMeasured.x = q.x();
    qMeasured.y = q.y();
    qMeasured.z = q.z();
    qMeasured.w = q.w();

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
    
    //if ( (fabs(rotationAxis[0]) < 0.0001) && (fabs(rotationAxis[1]) < 0.0001) && (fabs(rotationAxis[2]) < 0.0001) ){
    //    rotationAxis << 0.0 , 0.0 , 1.0;
    //};


    return rotationAxis;
};

double FlightController::GetAngleForDesiredRotation(Eigen::Vector3d firstVector, Eigen::Vector3d secondVector){
    /**
     * This function will calculate the angle between two vectors in 3d space, about an axis 
     * perpendicular to both the vectors. It uses the vector dot product for the same.
     */
    
    double rotationAngle;
    rotationAngle = std::acos(firstVector.dot(secondVector));
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
    //ROS_INFO_STREAM("Normalized thrust vector");
    //ROS_INFO_STREAM(normalizedThrustVector);
    
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
    m_desiredRotationMatrix = Eigen::AngleAxis<double>(rotationAngle,rotationAxis);
    //ROS_INFO_STREAM(desiredRotationMatrix);

    return m_desiredRotationMatrix;
};
Eigen::Matrix3d FlightController::GetErrorRotationMatrix(){
    /**
     * This function generates an error rotation matrix Rtilda. 
     * Rtilda = R * Rdtranspose
     */
    Eigen::Matrix3d m_desiredRotationMatrixTransposed = m_desiredRotationMatrix.transpose(); 
    m_errorRotationMatrix = m_currentRotationMatrix * m_desiredRotationMatrixTransposed;
    
    return m_errorRotationMatrix;

};

Eigen::Quaterniond FlightController::GetErrorQuaternion(){
    /**
     * This function returns the quaternion form of the error rotation matrix Rtilda
     * Rtilda = R * Rdtranspose
     */
    Eigen::Quaterniond q(m_errorRotationMatrix);
    m_errorQuaternion = q;
    return m_errorQuaternion;
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

Eigen::Matrix3d FlightController::GetSkewSymmetricMatrix(Eigen::Vector3d inputVector){
    /**
     * This function returns a skew symmetric matrix of an input vector.
     * For an input vector ux, uy ,uz  The skew symmetric matrix is of the form 
     * [0 -uz uy]
     * [uz 0 -ux]
     * [-uy ux 0]
     */
    double ux, uy, uz;
    ux = inputVector[0];
    uy = inputVector[1];
    uz = inputVector[2];



    Eigen::Matrix3d skewSymmetricMatrix;
    skewSymmetricMatrix << 0.0,     -uz,        uy,
                            uz,     0.0,       -ux,
                           -uy,      ux,       0.0;

    return skewSymmetricMatrix;
};

double FlightController::NormalizeAngle(double angle){
    
    double tanAngle = std::tan(angle);
    double angleNormalized;
    if (angle > 0.0){
        angleNormalized = std::atan2(tanAngle,1.0);
    }
    else if (angle < 0.0)
    {
        angleNormalized = std::atan2(tanAngle,-1.0);
    }
    else
    {
        return 0.0;
    };

    return angleNormalized;

};

Eigen::Vector3d FlightController::GetDesiredAngularVelocity(){
    /**
     * This method uses the current rotation matrix and the desired rotation matrix to 
     * generate a vector of desired angular velocity
     */

    // Generating an axis angle representation of the error rotation matrix.
    Eigen::AngleAxisd errorRotationAxisAngle(m_errorRotationMatrix);
    
    // Differentiating the angle deviation to get the angular velocity
    //ROS_INFO_STREAM("The error angle is ");
    //ROS_INFO_STREAM(errorRotationAxisAngle.angle());
    
    errorRotationAxisAngle.angle() = errorRotationAxisAngle.angle() / m_loopTime;
    //ROS_INFO_STREAM("The error Axis is ");
    //ROS_INFO_STREAM(errorRotationAxisAngle.axis());
    

    double omega_x, omega_y, omega_z; // Angular velocity components
    omega_x = errorRotationAxisAngle.angle() * errorRotationAxisAngle.axis()[0];
    omega_y = errorRotationAxisAngle.angle() * errorRotationAxisAngle.axis()[1];
    omega_z = errorRotationAxisAngle.angle() * errorRotationAxisAngle.axis()[2];

    omega_x = std::min(omega_x,2.0);
    omega_x = std::max(omega_x,-2.0);

    omega_y = std::min(omega_y,2.0);
    omega_y = std::max(omega_y,-2.0);

    omega_z = std::min(omega_y,2.0);
    omega_z = std::max(omega_y,-2.0);
    
    
    m_angularVelocityDesired << -omega_x, omega_y, omega_z;

    return m_angularVelocityDesired;
};

void FlightController::CalculateTorque(){
    /** This method uses a non-linear control equation with 11 terms to calculate the 
     * Torque vector
     */

    this->GetDesiredRotationMatrix();
    this->GetErrorRotationMatrix();
    this->GetDesiredAngularVelocity();
    this->GetErrorQuaternion();

    // First Term 
    Eigen::Vector3d firstTerm;
    Eigen::Matrix3d skewSymmetricAngularVelocity;
    skewSymmetricAngularVelocity = this->GetSkewSymmetricMatrix(m_angularVelocity);
    Eigen::Matrix3d identity3;
    identity3.setIdentity();
    firstTerm = skewSymmetricAngularVelocity * identity3 * m_angularVelocity;

    // Second Term
    // The second terms involves calculating the gyroscopic torque - we will leave this for now. 

    // Third term
    //// The derivative of desired angular velocity is calculated as the difference of desired angular vel and the current angular vel.
    Eigen::Vector3d thirdTerm;
    thirdTerm = (m_angularVelocityDesired - m_angularVelocity) / m_loopTime;
    
    // Fourth Term
    Eigen::Vector3d fourthTerm;
    Eigen::Vector3d sigma;
    Eigen::Vector3d angularVelocityError; //< omega tilda
    Eigen::Vector3d angularVelocityErrorVirtual; //< omega tilda v
    Eigen::Vector3d quaternionVector;
    
    
    angularVelocityError = m_desiredRotationMatrix * (m_angularVelocity - m_angularVelocityDesired);
    quaternionVector << m_errorQuaternion.x() , m_errorQuaternion.y() , m_errorQuaternion.z();
    angularVelocityErrorVirtual = - 2 * m_tuningParams.k_eta * m_errorQuaternion.w() * quaternionVector;
    sigma = angularVelocityError - angularVelocityErrorVirtual; 

    fourthTerm =  m_tuningParams.k_sigma * sigma;
    
    // Fifth Term 
    Eigen::Vector3d fifthTerm;
    fifthTerm = m_errorQuaternion.w() * quaternionVector / 2;

    // Sixth Term`
    Eigen::Vector3d sixthTerm;
    sixthTerm = m_tuningParams.k_eta * quaternionVector.transpose() * angularVelocityError * quaternionVector;

    // Seventh Term
    Eigen::Matrix3d seventhTerm;
    seventhTerm = m_desiredRotationMatrix * this->GetSkewSymmetricMatrix(m_angularVelocityDesired) * m_desiredRotationMatrix.transpose();
    
    // Eigth Term
    Eigen::Matrix3d eightTerm;
    eightTerm = m_tuningParams.k_eta * m_errorQuaternion.w() * m_errorQuaternion.w() * identity3;
    
    // Ninth Term 
    Eigen::Matrix3d ninthTerm;
    ninthTerm = m_tuningParams.k_eta * m_errorQuaternion.w() * this->GetSkewSymmetricMatrix(quaternionVector);
    
    
    Eigen::Vector3d tenthTerm;
    tenthTerm = (seventhTerm + eightTerm + ninthTerm) * angularVelocityError;
    
    // Final Equation
    m_torqueVector = firstTerm + (m_rotationalInertia / m_tuningParams.gamma ) * ( thirdTerm + 
                    m_desiredRotationMatrix.transpose() * (- fourthTerm - fifthTerm + sixthTerm - 
                     tenthTerm) );

    
    return;
};

void FlightController::CalculateMotorRPM(){
    /**
     * Use the torque vector and the thrust value to calculate the motor rpms
     * The method of doing so is simple least squares solving of an equation 
     * A X = B
     */

    // TODO : Verify the units of thrust and torque constants
    // TODO : Make sure you actually command rad/s and not rpm
    Eigen::Matrix4d A;
    double b , d , k;
    b = m_motorThrustConstant;
    d = m_propellerOffset;
    k = m_motorTorqueConstant;
    A << b , b , b , b ,
         0 ,d*b, 0 ,-d*b,
         d*b, 0 ,-d*b, 0 ,
         -k , k , -k , k ;
    
    Eigen::Vector4d B;
    B[0] = m_thrust;
    B[1] = m_torqueVector[0];
    B[2] = m_torqueVector[1];
    B[3] = m_torqueVector[2];
    //ROS_INFO_STREAM("The torque vector is ");
    //ROS_INFO_STREAM(m_torqueVector);
    Eigen::Vector4d X;
    X = A.colPivHouseholderQr().solve(B);
    m_motorCommands.motor1 = std::sqrt(X[0]);
    m_motorCommands.motor2 = std::sqrt(X[1]);
    m_motorCommands.motor3 = std::sqrt(X[2]);
    m_motorCommands.motor4 = std::sqrt(X[3]);

    if ( std::isnan(m_motorCommands.motor1) || std::isnan(m_motorCommands.motor2) || std::isnan(m_motorCommands.motor3) || std::isnan(m_motorCommands.motor4)){
        ROS_WARN("Invalid motor command ");
        return;
    };
    
    m_motorCommands.motor1 = std::min(m_motorCommands.motor1,1256.0);
    m_motorCommands.motor2 = std::min(m_motorCommands.motor2,1256.0);
    m_motorCommands.motor3 = std::min(m_motorCommands.motor3,1256.0);
    m_motorCommands.motor4 = std::min(m_motorCommands.motor4,1256.0);

    m_motorCommands.motor1 = std::max(m_motorCommands.motor1,0.0);
    m_motorCommands.motor2 = std::max(m_motorCommands.motor2,0.0);
    m_motorCommands.motor3 = std::max(m_motorCommands.motor3,0.0);
    m_motorCommands.motor4 = std::max(m_motorCommands.motor4,0.0);

    return;  
}

void FlightController::CommandMotorRPM(){
    /** 
     * This function is used to publish the current motor rpms
     */
    mav_msgs::ActuatorsPtr rpmCommand(new mav_msgs::Actuators);
    if ( std::isnan(m_motorCommands.motor1) || std::isnan(m_motorCommands.motor2) || std::isnan(m_motorCommands.motor3) || std::isnan(m_motorCommands.motor4)){
        ROS_WARN("Invalid motor command ");
        return;
    };
    
    rpmCommand->angular_velocities.clear();
    rpmCommand->angular_velocities.push_back(m_motorCommands.motor1);
    rpmCommand->angular_velocities.push_back(m_motorCommands.motor2);
    rpmCommand->angular_velocities.push_back(m_motorCommands.motor3);
    rpmCommand->angular_velocities.push_back(m_motorCommands.motor4);
    rpmCommand->header.stamp = ros::Time::now();

    motorCommandPublisher.publish(rpmCommand);

    return;
};

void FlightController::DebugPublisher(){
    /**
     * This method publishes a variety of topics in order to debug the working of the control
     * algorithm
     */

    // Publishing the Error Vector
    geometry_msgs::PoseStamped rotationError;
    rotationError.pose.position.x = 0.0;
    rotationError.pose.position.y = 0.0;
    rotationError.pose.position.z = 0.0; 
    rotationError.pose.orientation.x = m_errorQuaternion.x();
    rotationError.pose.orientation.y = m_errorQuaternion.y();
    rotationError.pose.orientation.z = m_errorQuaternion.z();
    rotationError.pose.orientation.w = m_errorQuaternion.w();
    rotationError.header.frame_id = "world";
    rotationError.header.stamp = ros::Time::now();
    debugErrorPublisher.publish(rotationError);

    // Publishing the desired rotation 
    Eigen::Quaterniond desiredRotationQuaternion(m_desiredRotationMatrix);
    geometry_msgs::PoseStamped desiredRotation;
    desiredRotation.pose.position.x = 0.0;
    desiredRotation.pose.position.y = 0.0;
    desiredRotation.pose.position.z = 0.0;
    desiredRotation.pose.orientation.x = desiredRotationQuaternion.x();
    desiredRotation.pose.orientation.y = desiredRotationQuaternion.y();
    desiredRotation.pose.orientation.z = desiredRotationQuaternion.z();
    desiredRotation.pose.orientation.w = desiredRotationQuaternion.w();
    desiredRotation.header.frame_id = "world";
    desiredRotation.header.stamp = ros::Time::now();
    debugDesiredRotationPublisher.publish(desiredRotation);

    // Publishing the desired angular velocity
    Eigen::Vector3d xAxis,yAxis,zAxis; 
    xAxis << 1.0 , 0.0 , 0.0;
    yAxis << 0.0 , 1.0 , 0.0;
    zAxis << 0.0 , 0.0 , 1.0;
    Eigen::Matrix3d desiredAngularVelocityMatrix;
    desiredAngularVelocityMatrix = Eigen::AngleAxisd(m_angularVelocityDesired[0],zAxis) * Eigen::AngleAxisd(m_angularVelocityDesired[1],yAxis)*
    Eigen::AngleAxisd(m_angularVelocityDesired[2],xAxis);  
    Eigen::Quaterniond desiredAngularVelocityQuaternion(desiredAngularVelocityMatrix);

    geometry_msgs::PoseStamped desiredAngularVelocity;
    desiredAngularVelocity.pose.position.x = 0.0;
    desiredAngularVelocity.pose.position.y = 0.0;
    desiredAngularVelocity.pose.position.z = 0.0;
    desiredAngularVelocity.pose.orientation.x = desiredAngularVelocityQuaternion.x();
    desiredAngularVelocity.pose.orientation.y = desiredAngularVelocityQuaternion.y();
    desiredAngularVelocity.pose.orientation.z = desiredAngularVelocityQuaternion.z();
    desiredAngularVelocity.pose.orientation.w = desiredAngularVelocityQuaternion.w();
    desiredAngularVelocity.header.stamp = ros::Time::now();
    desiredAngularVelocity.header.frame_id = "world";
    debugDesiredAngularVelocityPublisher.publish(desiredAngularVelocity);

    // Publising the thrust vector
    Eigen::Matrix3d thrustVectorMatrix;
    Eigen::Vector3d thrustVectorNormalized;
    thrustVectorNormalized = m_thrustVector.normalized();
    thrustVectorMatrix = Eigen::AngleAxisd(thrustVectorNormalized[2],zAxis) * Eigen::AngleAxisd(thrustVectorNormalized[1],yAxis)*
    Eigen::AngleAxisd(thrustVectorNormalized[0],xAxis);  
    Eigen::Quaterniond thrustVectorQuaternion(thrustVectorMatrix);

    geometry_msgs::PoseStamped thrustVector;
    thrustVector.pose.position.x = 0.0;
    thrustVector.pose.position.y = 0.0;
    thrustVector.pose.position.z = 0.0;
    thrustVector.pose.orientation.x = thrustVectorQuaternion.x();
    thrustVector.pose.orientation.y = thrustVectorQuaternion.y();
    thrustVector.pose.orientation.z = thrustVectorQuaternion.z();
    thrustVector.pose.orientation.w = thrustVectorQuaternion.w();
    thrustVector.header.frame_id = "world";
    thrustVector.header.stamp = ros::Time::now();
    debugThrustVectorPublisher.publish(thrustVector);

    // Publising the thrust vector
    Eigen::Matrix3d torqueVectorMatrix;
    Eigen::Vector3d torqueVectorNormalized;
    torqueVectorNormalized = m_torqueVector.normalized();
    torqueVectorMatrix = Eigen::AngleAxisd(torqueVectorNormalized[2],zAxis) * Eigen::AngleAxisd(torqueVectorNormalized[1],yAxis)*
    Eigen::AngleAxisd(torqueVectorNormalized[0],xAxis);  
    Eigen::Quaterniond torqueVectorQuaternion(torqueVectorMatrix);

    geometry_msgs::PoseStamped torqueVector;
    torqueVector.pose.position.x = 0.0;
    torqueVector.pose.position.y = 0.0;
    torqueVector.pose.position.z = 0.0;
    torqueVector.pose.orientation.x = torqueVectorQuaternion.x();
    torqueVector.pose.orientation.y = torqueVectorQuaternion.y();
    torqueVector.pose.orientation.z = torqueVectorQuaternion.z();
    torqueVector.pose.orientation.w = torqueVectorQuaternion.w();
    torqueVector.header.frame_id = "world";
    torqueVector.header.stamp = ros::Time::now();
    debugTorqueVectorPublisher.publish(torqueVector);

    return;
};


/** TODO 
 * 1. Calculate the gyroscopic torque
 * 5. Write a method for calculating motor RPMs out of thrust and torque.
 * 6. Test the error quaternion function using the Frobenius norm method.
 * 7. Write a test to ensure that the skew symmetric matrix is proper visually.
 * 
 * Quantities needed for rotational convergence
 * 
*/ 


int main(int argc, char **argv){
    // Initialize the node 
    
    ros::init(argc,argv,"flight_control");
    ros::NodeHandle nh;
    FlightController HummingBirdController(&nh);
    ros::Rate r(200);
    while(ros::ok()){
        HummingBirdController.CalculateThrust();
        HummingBirdController.CalculateTorque();
        HummingBirdController.CalculateMotorRPM();
        HummingBirdController.CommandMotorRPM();
        HummingBirdController.DebugPublisher();
        ROS_INFO("The commanded thrust is %f",HummingBirdController.m_thrust);
        ROS_INFO_STREAM(HummingBirdController.m_thrustVector);

        //ROS_INFO_STREAM("The commanded torque vector is ");
        //ROS_INFO_STREAM(HummingBirdController.m_torqueVector);
        r.sleep();
        ros::spinOnce();    
    };
    
    

}