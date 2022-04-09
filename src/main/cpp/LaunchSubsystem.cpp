#include "LaunchSubsystem.h"

#include <frc/smartdashboard/SmartDashboard.h>

LaunchSubsystem::LaunchSubsystem(
        int enableButtonIndex,
        rev::CANSparkMax& launchDrive,
        frc::Joystick& stick
) :
    m_runLaunch(false),
    m_buttonIndex(enableButtonIndex),
    m_launchDrive{launchDrive},
    m_stick{stick}
{
  m_launchDrive.RestoreFactoryDefaults();
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
    m_launchDrive.RestoreFactoryDefaults();
    /**
     * Parameters can be set by calling the appropriate Set method on the CANSparkMax object
     * whose properties you want to change
     * 
     * Set methods will return one of three REVLibError values which will let you know if the 
     * parameter was successfully set:
     *  REVLibError::kOk
     *  REVLibError::kError
     *  REVLibError::kTimeout
     */
    if(m_launchDrive.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast) != rev::REVLibError::kOk) {
      frc::SmartDashboard::PutString("Launch Idle Mode", "Error");
    }

    // Set ramp rate to 0
    if(m_launchDrive.SetOpenLoopRampRate(0) != rev::REVLibError::kOk) {
      frc::SmartDashboard::PutString("Launch Ramp Rate", "Error");
    }

    // read back ramp rate value
    frc::SmartDashboard::PutNumber("Launch Ramp Rate", m_launchDrive.GetOpenLoopRampRate());

    // Display local member values.
    frc::SmartDashboard::PutBoolean("Run Launch", m_runLaunch);
    frc::SmartDashboard::PutNumber("Launch RPM", m_launchEncoder.GetVelocity());
}

bool LaunchSubsystem::RunAutonomous(bool enabled)
{

  if (enabled)
  {
    // m_launchDrive.Set(-m_stick.GetThrottle());
    // Super-awesome calibration.
    m_launchDrive.Set(0.687);
  } else {
    m_launchDrive.Set(0);
  }

    // periodically read voltage, temperature, and applied output and publish to SmartDashboard
    frc::SmartDashboard::PutNumber("Launch Output", m_launchDrive.GetAppliedOutput());
    frc::SmartDashboard::PutNumber("Launch RPM", m_launchEncoder.GetVelocity());
    frc::SmartDashboard::PutBoolean("Run Launch", enabled);
    return enabled;
}

bool LaunchSubsystem::RunPeriodic()
{
    // Toggle Launch state on button press.
    if (m_stick.GetRawButtonPressed(m_buttonIndex))
    {
      m_runLaunch = !m_runLaunch;
    }
    // 
    if (m_runLaunch)
    {
    // Throttle is connected the slider on the controller.
    // The throttle axis reads -1.0 when pressed forward.
    // Launch motor is inverted from launch motor.
      m_launchDrive.Set(-m_stick.GetThrottle());
    } else {
      m_launchDrive.Set(0);
    }
    // periodically read voltage, temperature, and applied output and publish to SmartDashboard
    frc::SmartDashboard::PutNumber("Launch Output", m_launchDrive.GetAppliedOutput());
    frc::SmartDashboard::PutNumber("Launch RPM", m_launchEncoder.GetVelocity());
    frc::SmartDashboard::PutBoolean("Run Launch", m_runLaunch);
    return m_runLaunch;
}


void LaunchSubsystem::StopMotor()
{
    m_runLaunch = false;
    m_launchDrive.StopMotor();
}
