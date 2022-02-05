#include "IntakeSubsystem.h"

IntakeSubsystem::IntakeSubsystem(
        int enableButtonIndex,
        rev::CANSparkMax& intakeDrive,
        frc::Joystick& stick
) :
    m_runIntake(false),
    m_buttonIndex(enableButtonIndex),
    m_intakeDrive{intakeDrive},
    m_stick{stick}
{}


void IntakeSubsystem::Init()
{
    StopMotor();
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
    return m_runIntake;
}


void IntakeSubsystem::StopMotor()
{
    m_runIntake = false;
    m_intakeDrive.StopMotor();
}
