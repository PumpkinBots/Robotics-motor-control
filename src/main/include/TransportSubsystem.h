#pragma once

#include <frc/Joystick.h>

#include "rev/CANSparkMax.h"

class TransportSubsystem {
public:
    TransportSubsystem(
        int enableButtonIndex,
        rev::CANSparkMax& TransportDrive,
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
    int buttonIndex() const {return m_buttonIndex;}

private:
    const int m_buttonIndex;
    // Non-owning reference to the motor controller.
    rev::CANSparkMax& m_transportDrive;
    // Non-owning reference to the joystick.
    frc::Joystick& m_stick;
    // Encoder object created to display velocity values
    //rev::SparkMaxRelativeEncoder m_Transportencoder = m_TransportDrive.GetEncoder();
};