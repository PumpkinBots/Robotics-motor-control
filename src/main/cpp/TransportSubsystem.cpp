#include "TransportSubsystem.h"

TransportSubsystem::TransportSubsystem(
        int enableButtonIndex,
        rev::CANSparkMax& TransportDrive,
        frc::Joystick& stick
) :
    m_runTransport(false),
    m_buttonIndex(enableButtonIndex),
    m_TransportDrive{TransportDrive},
    m_stick{stick}
{
  m_TransportDrive.RestoreFactoryDefaults();
}


void TransportSubsystem::ModeInit()
{
    // Call GetRawButtonPressed to discard any button presses
    // made while the robot was disabled.
    m_stick.GetRawButtonPressed(m_buttonIndex);
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
    m_TransportDrive.RestoreFactoryDefaults();
}


bool TransportSubsystem::RunPeriodic()
{
    // Toggle Intake state on button press. 
    if (m_stick.GetRawButtonPressed(m_buttonIndex))
    {
      m_runTransport = !m_runTransport;
    }
    // 
    if (m_runTransport)
    {
    // Throttle is connected the slider on the controller.
    // The throttle axis reads -1.0 when pressed forward.
      m_TransportDrive.Set(-m_stick.GetThrottle());
    } else {
      m_TransportDrive.Set(0);
    }
    return m_runTransport;
}


void TransportSubsystem::StopMotor()
{
    m_runTransport = false;
    m_TransportDrive.StopMotor();
}
