#include "ClimbSubsystem.h"

#include <frc/smartdashboard/SmartDashboard.h>

ClimbSubsystem::ClimbSubsystem(
        rev::CANSparkMax& climbDrive,
        frc::XboxController& stick
) :
    m_drive{climbDrive},
    m_xbox{stick}
{
  m_drive.RestoreFactoryDefaults();
}


void ClimbSubsystem::ModeInit()
{
    StopMotor();
}

// This is mostly copied from SPARK-MAX-Examples/C++/Get and Set Parameters
void ClimbSubsystem::RobotInit()
{
    /**
     * The RestoreFactoryDefaults method can be used to reset the configuration parameters
     * in the SPARK MAX to their factory default state. If no argument is passed, these
     * parameters will not persist between power cycles
     */
    m_drive.RestoreFactoryDefaults();

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
    if(m_drive.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake) != rev::REVLibError::kOk) {
      frc::SmartDashboard::PutString("Climber Idle Mode", "Error");
    }

    /**
     * Similarly, parameters will have a Get method which allows you to retrieve their values
     * from the controller
     */
    if(m_drive.GetIdleMode() == rev::CANSparkMax::IdleMode::kCoast) {
      frc::SmartDashboard::PutString("Climber Idle Mode", "Coast");
    } else {
      frc::SmartDashboard::PutString("Climber Idle Mode", "Brake");
    }

    // Set ramp rate to 0
    if(m_drive.SetOpenLoopRampRate(0) != rev::REVLibError::kOk) {
      frc::SmartDashboard::PutString("Climber Ramp Rate", "Error");
    }

    // read back ramp rate value
    frc::SmartDashboard::PutNumber("Climber Ramp Rate", m_drive.GetOpenLoopRampRate());

  frc::SmartDashboard::PutNumber("Climber Output", m_drive.GetAppliedOutput());
}


void ClimbSubsystem::RunPeriodic()
{
  // Just hook up the climber motor to the right thumb stick.
  m_drive.Set(-m_xbox.GetRightY());
  frc::SmartDashboard::PutNumber("Climber Output", m_drive.GetAppliedOutput());
}


void ClimbSubsystem::StopMotor()
{
    m_drive.StopMotor();
}
