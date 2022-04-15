#pragma once

#include <frc/XboxController.h>

#include "rev/CANSparkMax.h"

class ClimbSubsystem {
public:
    ClimbSubsystem(
        rev::CANSparkMax& climbDrive,
        frc::XboxController& stick
    );

    // Initialize the subsystem from Robot::RobotInit().
    void RobotInit();
    // Call this when entering any mode.
    void ModeInit();
    // Call this in Periodic() function to check button and set motor.
    void RunPeriodic();

    // Stop the motor and disable run state.
    void StopMotor();

private:
    // Non-owning reference to the motor controller.
    rev::CANSparkMax& m_drive;
    // Non-owning reference to the joystick.
    frc::XboxController& m_xbox;
    // Encoder object created to display velocity values
    rev::SparkMaxRelativeEncoder m_encoder = m_drive.GetEncoder();
};
