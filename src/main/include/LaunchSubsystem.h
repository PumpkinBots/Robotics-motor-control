#pragma once

#include <frc/XboxController.h>
#include "rev/CANSparkMax.h"

class LaunchSubsystem {
public:
    LaunchSubsystem(
        rev::CANSparkMax& launchDrive,
        frc::XboxController& stick
    );

    // Initialize the subsystem from Robot::RobotInit().
    void RobotInit();
    // Call this when entering any mode.
    void ModeInit();
    // Call this in Periodic() function to check button and set motor.
    bool RunPeriodic();
    // Call this in Autonomous Periodic() function.
    // enabled: run the drive if true else stop.
    bool RunAutonomous(bool enabled);

    // Stop the motor and disable run state.
    void StopMotor();

public:
    // These are for tests an inspection.
    bool isEnabled() const {return m_runLaunch;}
    bool SetEnable(bool value)
    {
        m_runLaunch = value;
        return isEnabled();
    }

private:
    bool m_runLaunch;
    // Non-owning reference to the motor controller.
    rev::CANSparkMax& m_launchDrive;
    // Non-owning reference to the joystick.
    frc::XboxController& m_xbox;
    // Encoder object created to display velocity values
    rev::SparkMaxRelativeEncoder m_launchEncoder = m_launchDrive.GetEncoder();
};