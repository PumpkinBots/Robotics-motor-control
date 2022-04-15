#include "IntakeSubsystem.h"

#include <frc/smartdashboard/SmartDashboard.h>

IntakeSubsystem::IntakeSubsystem(
        rev::CANSparkMax& intakeDrive,
        frc::XboxController& stick
) :
    m_intakeDrive{intakeDrive},
    m_xbox{stick}
{
  m_intakeDrive.RestoreFactoryDefaults();
}


void IntakeSubsystem::ModeInit()
{
    // Call GetRawButtonPressed to discard any button presses
    // made while the robot was disabled.
    StopMotor();
    // Display local member values.
    frc::SmartDashboard::PutBoolean("Run Intake", false);
}

// This is mostly copied from SPARK-MAX-Examples/C++/Get and Set Parameters
void IntakeSubsystem::RobotInit()
{
    /**
     * The RestoreFactoryDefaults method can be used to reset the configuration parameters
     * in the SPARK MAX to their factory default state. If no argument is passed, these
     * parameters will not persist between power cycles
     */
    m_intakeDrive.RestoreFactoryDefaults();

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
    if(m_intakeDrive.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast) != rev::REVLibError::kOk) {
      frc::SmartDashboard::PutString("Intake Idle Mode", "Error");
    }

    /**
     * Similarly, parameters will have a Get method which allows you to retrieve their values
     * from the controller
     */
    if(m_intakeDrive.GetIdleMode() == rev::CANSparkMax::IdleMode::kCoast) {
      frc::SmartDashboard::PutString("Intake Idle Mode", "Coast");
    } else {
      frc::SmartDashboard::PutString("Intake Idle Mode", "Brake");
    }

    // Set ramp rate to 0
    if(m_intakeDrive.SetOpenLoopRampRate(0) != rev::REVLibError::kOk) {
      frc::SmartDashboard::PutString("Intake Ramp Rate", "Error");
    }

    // read back ramp rate value
    frc::SmartDashboard::PutNumber("Intake Ramp Rate", m_intakeDrive.GetOpenLoopRampRate());
}


bool IntakeSubsystem::RunPeriodic()
{
    // Run Intake on button press. Momentary press is easier to control.
    // The triggers return [0,1], so ignore small values.
    bool runIntake = m_xbox.GetLeftTriggerAxis() > 0.1;
    if (runIntake)
    {
      // Empirically determined input value.
      m_intakeDrive.Set(-0.65);
    } else {
      m_intakeDrive.Set(0);
    }
    // periodically read voltage, temperature, and applied output and publish to SmartDashboard
    frc::SmartDashboard::PutNumber("Intake Output", m_intakeDrive.GetAppliedOutput());
    frc::SmartDashboard::PutBoolean("Run Intake", runIntake);
    return runIntake;
}


void IntakeSubsystem::StopMotor()
{
  m_intakeDrive.StopMotor();
}
