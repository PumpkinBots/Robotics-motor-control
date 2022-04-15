#pragma once

#include <frc/Joystick.h>
#include <frc/XboxController.h>
#include "rev/CANSparkMax.h"

class IntakeRetractionSubsystem {
public:
    IntakeRetractionSubsystem(
        int enableButtonIndex,
        rev::CANSparkMax& intakeRetractionDrive,
        frc::XboxController& xbox
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
    bool isEnabled() const {return m_runIntakeRetraction;}
    int buttonIndex() const {return m_buttonIndex;}

    bool SetEnable(bool value)
    {
        m_runIntakeRetraction = value;
        return isEnabled();
    }

private:
    bool m_runIntakeRetraction;
    const int m_buttonIndex;
    // Non-owning reference to the motor controller.
    rev::CANSparkMax& m_intakeRetractionDrive;
    // Non-owning reference to the joystick.
    frc::XboxController& m_xbox;
    // Encoder object created to display velocity values
    rev::SparkMaxRelativeEncoder m_intakeRetractionEncoder = m_intakeRetractionDrive.GetEncoder();
};