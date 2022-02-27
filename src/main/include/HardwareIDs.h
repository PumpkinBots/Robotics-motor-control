#pragma once


/**
 * Can ID's for devices on the robot.
 * IDs 1-4 are reservered for future use with drive motors
 * ID 0 is illegal.
 * ID 64 is max.
 */
static constexpr int kTransportDeviceID = 5;
static constexpr int kLaunchDeviceID = 6;
static constexpr int kIntakeDeviceID = 7;  // Spark for intake motor


/**
 * Joystick buttons
*/

static constexpr int kIntakeButton = 2;
