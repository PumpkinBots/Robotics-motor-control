#include "IntakeSubsystem.h"

#include <frc/smartdashboard/SmartDashboard.h>

IntakeSubsystem::IntakeSubsystem(
        int enableButtonIndex,
        rev::CANSparkMax& intakeDrive,
        frc::Joystick& stick
) :
    m_runIntake(false),
    m_buttonIndex(enableButtonIndex),
    m_intakeDrive{intakeDrive},
    m_stick{stick}
{
  m_intakeDrive.RestoreFactoryDefaults();
}


void IntakeSubsystem::ModeInit()
{
    // Call GetRawButtonPressed to discard any button presses
    // made while the robot was disabled.
    m_stick.GetRawButtonPressed(m_buttonIndex);
    StopMotor();
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
      frc::SmartDashboard::PutString("Idle Mode", "Error");
    }

    /**
     * Similarly, parameters will have a Get method which allows you to retrieve their values
     * from the controller
     */
    if(m_intakeDrive.GetIdleMode() == rev::CANSparkMax::IdleMode::kCoast) {
      frc::SmartDashboard::PutString("Idle Mode", "Coast");
    } else {
      frc::SmartDashboard::PutString("Idle Mode", "Brake");
    }

    // Set ramp rate to 0
    if(m_intakeDrive.SetOpenLoopRampRate(0) != rev::REVLibError::kOk) {
      frc::SmartDashboard::PutString("Ramp Rate", "Error");
    }

    // read back ramp rate value
    frc::SmartDashboard::PutNumber("Ramp Rate", m_intakeDrive.GetOpenLoopRampRate());

    // Display local member values.
    frc::SmartDashboard::PutBoolean("Run Intake", m_runIntake);
    frc::SmartDashboard::PutNumber("Throttle", m_stick.GetThrottle());
}


bool IntakeSubsystem::RunPeriodic()
{
    // Toggle Intake state on button press.
    if (m_stick.GetRawButtonPressed(m_buttonIndex))
    {
      m_runIntake = !m_runIntake;
    }
    // 
    if (m_runIntake)
    {
    // Throttle is connected the slider on the controller.
    // The throttle axis reads -1.0 when pressed forward.
      m_intakeDrive.Set(m_stick.GetThrottle());
    } else {
      m_intakeDrive.Set(0);
    }
    // periodically read voltage, temperature, and applied output and publish to SmartDashboard
    frc::SmartDashboard::PutNumber("Voltage", m_intakeDrive.GetBusVoltage());
    frc::SmartDashboard::PutNumber("Temperature", m_intakeDrive.GetMotorTemperature());
    frc::SmartDashboard::PutNumber("Output", m_intakeDrive.GetAppliedOutput());
    frc::SmartDashboard::PutBoolean("Run Intake", m_runIntake);
    frc::SmartDashboard::PutNumber("Throttle", m_stick.GetThrottle());

    return m_runIntake;
}


void IntakeSubsystem::StopMotor()
{
    m_runIntake = false;
    m_intakeDrive.StopMotor();
}
