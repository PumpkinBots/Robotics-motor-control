#pragma once

#include <frc/Joystick.h>

#include "rev/CANSparkMax.h"

class IntakeSubsystem {
public:
    IntakeSubsystem(
        int enableButtonIndex,
        rev::CANSparkMax& intakeDrive,
        frc::Joystick& stick
    );

    // Initialize the subsystem from Robot::RobotInit().
    void RobotInit();
    // Call this when entering any mode.
    void ModeInit();
    // Call this in Periodic() function to check button and set motor.
    bool RunPeriodic();

    // Stop the motor and disable run state.
    void StopMotor();

public:
    // These are for tests an inspection.
    bool isEnabled() const {return m_runIntake;}
    int buttonIndex() const {return m_buttonIndex;}
    bool SetEnable(bool value)
    {
        m_runIntake = value;
        return isEnabled();
    }

private:
    bool m_runIntake;
    const int m_buttonIndex;
    // Non-owning reference to the motor controller.
    rev::CANSparkMax& m_intakeDrive;
    // Non-owning reference to the joystick.
    frc::Joystick& m_stick;
    // Encoder object created to display velocity values
    rev::SparkMaxRelativeEncoder m_encoder;
    double m_throttle;
};