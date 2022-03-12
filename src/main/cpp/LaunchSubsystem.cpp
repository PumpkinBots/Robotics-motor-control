#include "LaunchSubsystem.h"

LaunchSubsystem::LaunchSubsystem(
        int enableButtonIndex,
        rev::CANSparkMax& launchDrive,
        frc::Joystick& stick
) :
    m_runLaunch(false),
    m_buttonIndex(enableButtonIndex),
    m_LaunchDrive{launchDrive},
    m_stick{stick}
{
  m_LaunchDrive.RestoreFactoryDefaults();
}


void LaunchSubsystem::ModeInit()
{
    // Call GetRawButtonPressed to discard any button presses
    // made while the robot was disabled.
    m_stick.GetRawButtonPressed(m_buttonIndex);
    StopMotor();
}

// This is mostly copied from SPARK-MAX-Examples/C++/Get and Set Parameters
void LaunchSubsystem::RobotInit()
{
    /**
     * The RestoreFactoryDefaults method can be used to reset the configuration parameters
     * in the SPARK MAX to their factory default state. If no argument is passed, these
     * parameters will not persist between power cycles
     */
    m_LaunchDrive.RestoreFactoryDefaults();
}


bool LaunchSubsystem::RunPeriodic()
{
    // Toggle Intake state on button press. 
    if (m_stick.GetRawButtonPressed(m_buttonIndex))
    {
      m_runLaunch = !m_runLaunch;
    }
    // 
    if (m_runLaunch)
    {
    // Throttle is connected the slider on the controller.
    // The throttle axis reads -1.0 when pressed forward.
    // Launch motor is inverted from intake motor.
      m_LaunchDrive.Set(-m_stick.GetThrottle());
    } else {
      m_LaunchDrive.Set(0);
    }
    return m_runLaunch;
}


void LaunchSubsystem::StopMotor()
{
    m_runLaunch = false;
    m_LaunchDrive.StopMotor();
}
