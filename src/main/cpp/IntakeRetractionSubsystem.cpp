#include "IntakeRetractionSubsystem.h"

#include <frc/smartdashboard/SmartDashboard.h>

IntakeRetractionSubsystem::IntakeRetractionSubsystem(
        rev::CANSparkMax& intakeRetractionDrive,
        frc::XboxController& xbox
) :
    m_drive{intakeRetractionDrive},
    m_xbox{xbox}
{
  m_drive.RestoreFactoryDefaults();
}


void IntakeRetractionSubsystem::ModeInit()
{
    // Call GetRawButtonPressed to discard any button presses
    // made while the robot was disabled.

    StopMotor();
}

// This is mostly copied from SPARK-MAX-Examples/C++/Get and Set Parameters
void IntakeRetractionSubsystem::RobotInit()
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
      frc::SmartDashboard::PutString("Intake Retraction Idle Mode", "Error");
    }

    // Set ramp rate to 0
    if(m_drive.SetOpenLoopRampRate(0) != rev::REVLibError::kOk) {
      frc::SmartDashboard::PutString("Intake Retraction Ramp Rate", "Error");
    }
}


bool IntakeRetractionSubsystem::RunPeriodic()
{
    // Toggle Transport state on button press.
    bool runArmDown = m_xbox.GetLeftBumper();
    bool runArmUp = m_xbox.GetRightBumper();
    bool doRun = false;
    if (runArmDown && runArmUp) {
      // Input error, do nothing.
      m_drive.Set(0);
    } else if (runArmUp) {
      doRun = true;
      m_drive.Set(1.0);
    } else if (runArmDown) {
      doRun = true;
      m_drive.Set(-1.0);
    } else {
      m_drive.Set(0);
    }
    frc::SmartDashboard::PutNumber("Intake Retraction RPM", m_intakeRetractionEncoder.GetVelocity());
  
    return doRun;
}


void IntakeRetractionSubsystem::StopMotor()
{
    m_drive.StopMotor();
}
