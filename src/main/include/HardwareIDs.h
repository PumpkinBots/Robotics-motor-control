#pragma once


/**
 * Can ID's for devices on the robot.
 * IDs 1-4 are reservered for future use with drive motors
 * ID 0 is illegal.
 * ID 64 is max.
 */
static constexpr int kLaunchDeviceID = 5; // Spark for launcher motor
static constexpr int kTransportDeviceID = 6; // Spark for transport motor
static constexpr int kIntakeDeviceID = 7;  // Spark for intake motor

/**
 * Joystick buttons
*/
static constexpr int kIntakeButton = 2;
static constexpr int kTransportButton = 1;
static constexpr int kLaunchButton = 3;

