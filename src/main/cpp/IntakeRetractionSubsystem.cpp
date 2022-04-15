#include "IntakeRetractionSubsystem.h"

#include <frc/smartdashboard/SmartDashboard.h>

IntakeRetractionSubsystem::IntakeRetractionSubsystem(
        rev::CANSparkMax& intakeRetractionDrive,
        frc::XboxController& xbox
) :
    m_runIntakeRetraction(false),
    m_intakeRetractionDrive{intakeRetractionDrive},
    m_xbox{xbox}
{
  m_intakeRetractionDrive.RestoreFactoryDefaults();
}


void IntakeRetractionSubsystem::ModeInit()
{
    // Call GetRawButtonPressed to discard any button presses
    // made while the robot was disabled.

    StopMotor();
    // Display local member values.
    frc::SmartDashboard::PutBoolean("Run Intake", m_runIntakeRetraction);

}

// This is mostly copied from SPARK-MAX-Examples/C++/Get and Set Parameters
void IntakeRetractionSubsystem::RobotInit()
{
    /**
     * The RestoreFactoryDefaults method can be used to reset the configuration parameters
     * in the SPARK MAX to their factory default state. If no argument is passed, these
     * parameters will not persist between power cycles
     */
    m_intakeRetractionDrive.RestoreFactoryDefaults();

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
    if(m_intakeRetractionDrive.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast) != rev::REVLibError::kOk) {
      frc::SmartDashboard::PutString("Intake Idle Mode", "Error");
    }

    /**
     * Similarly, parameters will have a Get method which allows you to retrieve their values
     * from the controller
     */
    if(m_intakeRetractionDrive.GetIdleMode() == rev::CANSparkMax::IdleMode::kCoast) {
      frc::SmartDashboard::PutString("Intake Idle Mode", "Coast");
    } else {
      frc::SmartDashboard::PutString("Intake Idle Mode", "Brake");
    }

    // Set ramp rate to 0
    if(m_intakeRetractionDrive.SetOpenLoopRampRate(0) != rev::REVLibError::kOk) {
      frc::SmartDashboard::PutString("Intake Ramp Rate", "Error");
    }

    // read back ramp rate value
    frc::SmartDashboard::PutNumber("Intake Ramp Rate", m_intakeRetractionDrive.GetOpenLoopRampRate());

    // Display local member values.
    frc::SmartDashboard::PutBoolean("Run Intake Retraction", m_runIntakeRetraction);
}


bool IntakeRetractionSubsystem::RunPeriodic()
{
    // Toggle Transport state on button press.
    bool runIntakeRetraction = m_xbox.GetAButton();
    if (runIntakeRetraction)
    {
    // Throttle is connected the slider on the controller.
    // The throttle axis reads -1.0 when pressed forward.
      m_intakeRetractionDrive.Set(0.8);
    } else {
      m_intakeRetractionDrive.Set(0);
    }
    frc::SmartDashboard::PutBoolean("Run Intake Retraction", m_runIntakeRetraction);
    frc::SmartDashboard::PutNumber("Launch RPM", m_intakeRetractionEncoder.GetVelocity());
  
    return runIntakeRetraction;
}


void IntakeRetractionSubsystem::StopMotor()
{
    m_runIntakeRetraction = false;
    m_intakeRetractionDrive.StopMotor();
}
