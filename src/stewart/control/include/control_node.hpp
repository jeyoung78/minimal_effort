#ifndef CONTROL_NODE_HPP_
#define CONTROL_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "linux/i2c-dev.h"
#include "sys/ioctl.h"
#include <fcntl.h>
#include <unistd.h>
#include <iostream>
#include <cmath>

class ControlNode : public rclcpp::Node
{
public:
    ControlNode();

    const double NORM_VECTOR_THRESHOLD = 0.001; // Used for determining if unit vector

    const double NORMAL_Z = 0.9936;

    const double L = sqrt(2 * pow(0.15, 2)); // Distance between center of plate and edges

    const double HEIGHT = 0.125; // THIS VALUE IS VERY ARBITRARY, BUT DO NOT CHANGE BECAUSE IT MESSES UP OUR IK

    // l[0] is the distance between the center of the base and the motor
    // l[1] is the length of the 3D printed arm (1st linkage)
    // l[2] is the length of the threaded rod (2nd linkage)
    const double l[3] = {sqrt(2 * pow(0.12, 2)), 0.0635, 0.0926};

    // Configure pubsub nodes to keep last 30 messages.
    static constexpr int ADVERTISING_FREQ = 30;

    int file = 0;

    const char *device = "/dev/i2c-1";

/*

Servo Controller Definitions

*/
#define PCA9685_ADDRESS 0x40 // Default I2C address for PCA9685
#define MODE1 0x00
#define PRESCALE 0xFE

// Define registers for each PWM channel
#define LED0_ON_L 0x06
#define LED0_ON_H 0x07
#define LED0_OFF_L 0x08
#define LED0_OFF_H 0x09

private:
    void controlCallback(
        geometry_msgs::msg::Vector3 msg);

    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr norm_vector_sub_;

    geometry_msgs::msg::Vector3 checkValidNormVector(
        geometry_msgs::msg::Vector3 normVector);

    geometry_msgs::msg::Vector3 bPointIK(
        int pointNumber,
        geometry_msgs::msg::Vector3 normV);

    geometry_msgs::msg::Vector3 jointAngleIK(
        int jointNumber,
        geometry_msgs::msg::Vector3 bPoint);

    int angleTheta(
        int motorNumber,
        geometry_msgs::msg::Vector3 pPoint);

    int offsetThetaPulse(
        int motorNumber,
        int theta);

    int angleToPWM(
        int desiredAngle,
        int angleOffset);

    int angleToPWMReversed(
        int desiredAngle,
        int angleOffset);

    // Write a single byte to a specific register
    void writeByte(int file, uint8_t reg, uint8_t value);

    // Initialize the PCA9685
    void initPCA9685(int file);

    // Set PWM for a specific channel
    void setPWM(int file, int channel, int on, int off);
};

#endif
