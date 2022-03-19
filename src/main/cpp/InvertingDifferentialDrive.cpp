// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "InvertingDifferentialDrive.h"

#include <algorithm>
#include <cmath>

#include <hal/FRCUsageReporting.h>
#include <wpi/sendable/SendableBuilder.h>
#include <wpi/sendable/SendableRegistry.h>

#include "frc/MathUtil.h"
#include "frc/SpeedController.h"

using namespace frc::PumpkinBots;

#if defined(_MSC_VER)
#pragma warning(disable : 4996)  // was declared deprecated
#elif defined(__clang__)
#pragma clang diagnostic ignored "-Wdeprecated-declarations"
#elif defined(__GNUC__)
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif

InvertingDifferentialDrive::InvertingDifferentialDrive(frc::SpeedController& leftMotor,
                                     frc::SpeedController& rightMotor)
    : m_leftMotor(&leftMotor), m_rightMotor(&rightMotor) {
  wpi::SendableRegistry::AddChild(this, m_leftMotor);
  wpi::SendableRegistry::AddChild(this, m_rightMotor);
  static int instances = 0;
  ++instances;
  wpi::SendableRegistry::AddLW(this, "InvertingDifferentialDrive", instances);
}

void InvertingDifferentialDrive::ArcadeDrive(double xSpeed, double zRotation,
                                    bool squareInputs) {
  static bool reported = false;
  if (!reported) {
    HAL_Report(HALUsageReporting::kResourceType_RobotDrive,
               HALUsageReporting::kRobotDrive2_DifferentialArcade, 2);
    reported = true;
  }

  xSpeed = ApplyDeadband(xSpeed, m_deadband);
  zRotation = ApplyDeadband(zRotation, m_deadband);

  auto [left, right] = ArcadeDriveIK(xSpeed, zRotation, squareInputs);

  m_leftMotor->Set(left);
  m_rightMotor->Set(right);

  Feed();
}

void InvertingDifferentialDrive::CurvatureDrive(double xSpeed, double zRotation,
                                       bool allowTurnInPlace) {
  static bool reported = false;
  if (!reported) {
    HAL_Report(HALUsageReporting::kResourceType_RobotDrive,
               HALUsageReporting::kRobotDrive2_DifferentialCurvature, 2);
    reported = true;
  }

  xSpeed = ApplyDeadband(xSpeed, m_deadband);
  zRotation = ApplyDeadband(zRotation, m_deadband);

  auto [left, right] = CurvatureDriveIK(xSpeed, zRotation, allowTurnInPlace);

  m_leftMotor->Set(left * m_maxOutput);
  m_rightMotor->Set(right * m_maxOutput);

  Feed();
}

void InvertingDifferentialDrive::TankDrive(double leftSpeed, double rightSpeed,
                                  bool squareInputs) {
  static bool reported = false;
  if (!reported) {
    HAL_Report(HALUsageReporting::kResourceType_RobotDrive,
               HALUsageReporting::kRobotDrive2_DifferentialTank, 2);
    reported = true;
  }

  leftSpeed = ApplyDeadband(leftSpeed, m_deadband);
  rightSpeed = ApplyDeadband(rightSpeed, m_deadband);

  auto [left, right] = TankDriveIK(leftSpeed, rightSpeed, squareInputs);

  m_leftMotor->Set(left * m_maxOutput);
  m_rightMotor->Set(right * m_maxOutput);

  Feed();
}

InvertingDifferentialDrive::WheelSpeeds InvertingDifferentialDrive::ArcadeDriveIK(
    double xSpeed, double zRotation, bool squareInputs) {
  xSpeed = std::clamp(xSpeed, -1.0, 1.0);
  zRotation = std::clamp(zRotation, -1.0, 1.0);

  // Square the inputs (while preserving the sign) to increase fine control
  // while permitting full power.
  if (squareInputs) {
    xSpeed = std::copysign(xSpeed * xSpeed, xSpeed);
    zRotation = std::copysign(zRotation * zRotation, zRotation);
  }

  double leftSpeed;
  double rightSpeed;

  double maxInput =
      std::copysign(std::max(std::abs(xSpeed), std::abs(zRotation)), xSpeed);

  // Sign is used because `xSpeed >= 0.0` succeeds for -0.0
  if (!std::signbit(xSpeed)) {
    // First quadrant, else second quadrant
    if (!std::signbit(zRotation)) {
      leftSpeed = maxInput;
      rightSpeed = xSpeed - zRotation;
    } else {
      leftSpeed = xSpeed + zRotation;
      rightSpeed = maxInput;
    }
  } else {
    // Third quadrant, else fourth quadrant
    if (!std::signbit(zRotation)) {
      leftSpeed = xSpeed + zRotation;
      rightSpeed = maxInput;
    } else {
      leftSpeed = maxInput;
      rightSpeed = xSpeed - zRotation;
    }
  }

  // Normalize the wheel speeds
  double maxMagnitude = std::max(std::abs(leftSpeed), std::abs(rightSpeed));
  if (maxMagnitude > 1.0) {
    leftSpeed /= maxMagnitude;
    rightSpeed /= maxMagnitude;
  }

  return {leftSpeed, rightSpeed};
}

InvertingDifferentialDrive::WheelSpeeds InvertingDifferentialDrive::CurvatureDriveIK(
    double xSpeed, double zRotation, bool allowTurnInPlace) {
  xSpeed = std::clamp(xSpeed, -1.0, 1.0);
  zRotation = std::clamp(zRotation, -1.0, 1.0);

  double leftSpeed = 0.0;
  double rightSpeed = 0.0;

  if (allowTurnInPlace) {
    leftSpeed = xSpeed + zRotation;
    rightSpeed = xSpeed - zRotation;
  } else {
    leftSpeed = xSpeed + std::abs(xSpeed) * zRotation;
    rightSpeed = xSpeed - std::abs(xSpeed) * zRotation;
  }

  // Normalize wheel speeds
  double maxMagnitude = std::max(std::abs(leftSpeed), std::abs(rightSpeed));
  if (maxMagnitude > 1.0) {
    leftSpeed /= maxMagnitude;
    rightSpeed /= maxMagnitude;
  }

  return {leftSpeed, rightSpeed};
}

InvertingDifferentialDrive::WheelSpeeds InvertingDifferentialDrive::TankDriveIK(
    double leftSpeed, double rightSpeed, bool squareInputs) {
  leftSpeed = std::clamp(leftSpeed, -1.0, 1.0);
  rightSpeed = std::clamp(rightSpeed, -1.0, 1.0);

  // Square the inputs (while preserving the sign) to increase fine control
  // while permitting full power.
  if (squareInputs) {
    leftSpeed = std::copysign(leftSpeed * leftSpeed, leftSpeed);
    rightSpeed = std::copysign(rightSpeed * rightSpeed, rightSpeed);
  }

  return {leftSpeed, rightSpeed};
}

void InvertingDifferentialDrive::StopMotor() {
  m_leftMotor->StopMotor();
  m_rightMotor->StopMotor();
  Feed();
}

std::string InvertingDifferentialDrive::GetDescription() const {
  return "InvertingDifferentialDrive";
}

void InvertingDifferentialDrive::InitSendable(wpi::SendableBuilder& builder) {
  builder.SetSmartDashboardType("DifferentialDrive");
  builder.SetActuator(true);
  builder.SetSafeState([=] { StopMotor(); });
  builder.AddDoubleProperty(
      "Left Motor Speed", [=] { return m_leftMotor->Get(); },
      [=](double value) { m_leftMotor->Set(value); });
  builder.AddDoubleProperty(
      "Right Motor Speed", [=] { return m_rightMotor->Get(); },
      [=](double value) { m_rightMotor->Set(value); });
}
