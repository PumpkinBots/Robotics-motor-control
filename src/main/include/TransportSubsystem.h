#pragma once

#include <frc/XboxController.h>
#include "rev/CANSparkMax.h"

class TransportSubsystem {
public:
    TransportSubsystem(
        rev::CANSparkMax& TransportDrive,
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

private:
    // Non-owning reference to the motor controller.
    rev::CANSparkMax& m_transportDrive;
    // Non-owning reference to the joystick.
    frc::XboxController& m_xbox;
    // Encoder object created to display velocity values
    //rev::SparkMaxRelativeEncoder m_Transportencoder = m_TransportDrive.GetEncoder();
};