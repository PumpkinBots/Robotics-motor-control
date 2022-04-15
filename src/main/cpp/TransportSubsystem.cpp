#include "TransportSubsystem.h"

#include <frc/smartdashboard/SmartDashboard.h>

TransportSubsystem::TransportSubsystem(
        rev::CANSparkMax& TransportDrive,
        frc::XboxController& stick
) :
    m_transportDrive{TransportDrive},
    m_xbox{stick}
{
  m_transportDrive.RestoreFactoryDefaults();
}


void TransportSubsystem::ModeInit()
{
    // Call GetRawButtonPressed to discard any button presses
    // made while the robot was disabled.
    StopMotor();
}

// This is mostly copied from SPARK-MAX-Examples/C++/Get and Set Parameters
void TransportSubsystem::RobotInit()
{
    /**
     * The RestoreFactoryDefaults method can be used to reset the configuration parameters
     * in the SPARK MAX to their factory default state. If no argument is passed, these
     * parameters will not persist between power cycles
     */
    m_transportDrive.RestoreFactoryDefaults();

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
    if(m_transportDrive.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast) != rev::REVLibError::kOk) {
      frc::SmartDashboard::PutString("Transport Idle Mode", "Error");
    }

    /**
     * Similarly, parameters will have a Get method which allows you to retrieve their values
     * from the controller
     */
    if(m_transportDrive.GetIdleMode() == rev::CANSparkMax::IdleMode::kCoast) {
      frc::SmartDashboard::PutString("Transport Idle Mode", "Coast");
    } else {
      frc::SmartDashboard::PutString("Transport Idle Mode", "Brake");
    }

    // Set ramp rate to 0
    if(m_transportDrive.SetOpenLoopRampRate(0) != rev::REVLibError::kOk) {
      frc::SmartDashboard::PutString("Transport Ramp Rate", "Error");
    }

    // read back ramp rate value
    frc::SmartDashboard::PutNumber("Transport Ramp Rate", m_transportDrive.GetOpenLoopRampRate());

    // Display local member values.
    frc::SmartDashboard::PutBoolean("Run Transport", false);
}

bool TransportSubsystem::RunAutonomous(bool enabled)
{
  if (enabled)
  {
    // m_transportDrive.Set(-m_stick.GetThrottle());
    // Super-awesome calibration.
    m_transportDrive.Set(0.8);
  } else {
    m_transportDrive.Set(0);
  }

    // periodically read applied output and publish to SmartDashboard
    frc::SmartDashboard::PutNumber("Transport Output", m_transportDrive.GetAppliedOutput());
    frc::SmartDashboard::PutBoolean("Run Transport", enabled);
    return enabled;
}

bool TransportSubsystem::RunPeriodic()
{
    // Run Transport on momentary button press.
    // The triggers return [0,1], so ignore small values.
    bool runTransport = m_xbox.GetRightTriggerAxis() > 0.1;
    if (runTransport)
    {
      m_transportDrive.Set(0.65);
    } else {
      m_transportDrive.Set(0);
    }
    // periodically read applied output and publish to SmartDashboard
    frc::SmartDashboard::PutNumber("Transport Output", m_transportDrive.GetAppliedOutput());
    frc::SmartDashboard::PutBoolean("Run Transport", runTransport);
    return runTransport;
}


void TransportSubsystem::StopMotor()
{
    m_transportDrive.StopMotor();
}
