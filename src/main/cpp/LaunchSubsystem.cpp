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
    m_stick{stick},
    m_encoder{m_launchDrive.GetEncoder()},
    m_throttle{1.0}
{
  m_launchDrive.RestoreFactoryDefaults();
}


void LaunchSubsystem::ModeInit()
{
  // Call GetRawButtonPressed to discard any button presses
  // made while the robot was disabled.
  m_stick.GetRawButtonPressed(m_buttonIndex);
  StopMotor();
  // Display local member values.
  frc::SmartDashboard::PutBoolean("Run Launch", m_runLaunch);
  frc::SmartDashboard::PutNumber("Launch Throttle", m_throttle);
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

  /**
   * Similarly, parameters will have a Get method which allows you to retrieve their values
   * from the controller
   */
  if(m_launchDrive.GetIdleMode() == rev::CANSparkMax::IdleMode::kCoast) {
    frc::SmartDashboard::PutString("Launch Idle Mode", "Coast");
  } else {
    frc::SmartDashboard::PutString("Launch Idle Mode", "Brake");
  }

  // Set ramp rate to 0
  if(m_launchDrive.SetOpenLoopRampRate(0) != rev::REVLibError::kOk) {
    frc::SmartDashboard::PutString("Launch Ramp Rate", "Error");
  }

  // read back ramp rate value
  frc::SmartDashboard::PutNumber("Launch Ramp Rate", m_launchDrive.GetOpenLoopRampRate());

  // Display local member values.
  frc::SmartDashboard::PutBoolean("Run Launch", m_runLaunch);
  // Set Default if the value is not already set.
  frc::SmartDashboard::SetDefaultNumber("Launch Throttle", m_throttle);
  frc::SmartDashboard::SetPersistent("Launch Throttle");
  // Get current SmartDashboard throttle value.
  m_throttle = frc::SmartDashboard::GetNumber("Launch Throttle", 1.0);
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
    // Get current SmartDashboard throttle value.
    m_throttle = frc::SmartDashboard::GetNumber("Launch Throttle", 1.0);
    m_launchDrive.Set(m_throttle);
  } else {
    m_launchDrive.Set(0);
  }
  // periodically read voltage, temperature, and applied output and publish to SmartDashboard
  frc::SmartDashboard::PutNumber("Launch Voltage", m_launchDrive.GetBusVoltage());
  frc::SmartDashboard::PutNumber("Launch Temperature", m_launchDrive.GetMotorTemperature());
  frc::SmartDashboard::PutNumber("Launch Output", m_launchDrive.GetAppliedOutput());
  frc::SmartDashboard::PutBoolean("Run Launch", m_runLaunch);
  return m_runLaunch;
}


void LaunchSubsystem::StopMotor()
{
  m_runLaunch = false;
  m_launchDrive.StopMotor();
}
