#include "ClimbSubsystem.h"

#include <frc/smartdashboard/SmartDashboard.h>

ClimbSubsystem::ClimbSubsystem(
        int enableButtonIndex,
        rev::CANSparkMax& climbDrive,
        frc::Joystick& stick
) :
    m_runClimber(false),
    m_buttonIndex(enableButtonIndex),
    m_climbDrive{climbDrive},
    m_stick{stick}
{
  m_climbDrive.RestoreFactoryDefaults();
}


void ClimbSubsystem::ModeInit()
{
    // Call GetRawButtonPressed to discard any button presses
    // made while the robot was disabled.
    m_stick.GetRawButtonPressed(m_buttonIndex);

    StopMotor();
    // Display local member values.
    frc::SmartDashboard::PutBoolean("Run Climber", m_runClimber);
    frc::SmartDashboard::PutNumber("Throttle", m_stick.GetThrottle());

}

// This is mostly copied from SPARK-MAX-Examples/C++/Get and Set Parameters
void ClimbSubsystem::RobotInit()
{
    /**
     * The RestoreFactoryDefaults method can be used to reset the configuration parameters
     * in the SPARK MAX to their factory default state. If no argument is passed, these
     * parameters will not persist between power cycles
     */
    m_climbDrive.RestoreFactoryDefaults();

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
    if(m_climbDrive.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast) != rev::REVLibError::kOk) {
      frc::SmartDashboard::PutString("Intake Idle Mode", "Error");
    }

    /**
     * Similarly, parameters will have a Get method which allows you to retrieve their values
     * from the controller
     */
    if(m_climbDrive.GetIdleMode() == rev::CANSparkMax::IdleMode::kCoast) {
      frc::SmartDashboard::PutString("Intake Idle Mode", "Coast");
    } else {
      frc::SmartDashboard::PutString("Intake Idle Mode", "Brake");
    }

    // Set ramp rate to 0
    if(m_climbDrive.SetOpenLoopRampRate(0) != rev::REVLibError::kOk) {
      frc::SmartDashboard::PutString("Intake Ramp Rate", "Error");
    }

    // read back ramp rate value
    frc::SmartDashboard::PutNumber("Intake Ramp Rate", m_climbDrive.GetOpenLoopRampRate());

    // Display local member values.
    frc::SmartDashboard::PutBoolean("Run Intake", m_runClimber);
    frc::SmartDashboard::PutNumber("Throttle", m_stick.GetThrottle());
}


bool ClimbSubsystem::RunPeriodic()
{
    // Toggle Transport state on button press.
    bool runClimber = m_stick.GetRawButton(m_buttonIndex);
    if (runClimber)
    {
    // Throttle is connected the slider on the controller.
    // The throttle axis reads -1.0 when pressed forward.
      m_climbDrive.Set(m_stick.GetThrottle());
    } else {
      m_climbDrive.Set(0);
    }
    return runClimber;
}


void ClimbSubsystem::StopMotor()
{
    m_runClimber = false;
    m_climbDrive.StopMotor();
}
