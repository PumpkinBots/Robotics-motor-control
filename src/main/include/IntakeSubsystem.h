#pragma once

#include <frc/XboxController.h>
#include "rev/CANSparkMax.h"

class IntakeSubsystem {
public:
    IntakeSubsystem(
        rev::CANSparkMax& intakeDrive,
        frc::XboxController& stick
    );

    // Initialize the subsystem from Robot::RobotInit().
    void RobotInit();
    // Call this when entering any mode.
    void ModeInit();
    // Call this in Periodic() function to check button and set motor.
    bool RunPeriodic();

    // Stop the motor and disable run state.
    void StopMotor();

private:
    // Non-owning reference to the motor controller.
    rev::CANSparkMax& m_intakeDrive;
    // Non-owning reference to the joystick.
    frc::XboxController& m_xbox;
    // Encoder object created to display velocity values
    rev::SparkMaxRelativeEncoder m_encoder = m_intakeDrive.GetEncoder();
};