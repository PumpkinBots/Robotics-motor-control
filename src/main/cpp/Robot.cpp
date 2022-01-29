// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/Joystick.h>
#include <frc/TimedRobot.h>
#include <frc/Timer.h>
#include <frc/motorcontrol/PWMSparkMax.h>
#include <frc/motorcontrol/MotorControllerGroup.h>
#include <frc/drive/DifferentialDrive.h>

/**
 * This sample program shows how to control a motor using a joystick. In the
 * operator control part of the program, the joystick is read and the value is
 * written to the motor.
 *
 * Joystick analog values range from -1 to 1 and speed controller inputs as
 * range from -1 to 1 making it easy to work together.
 */
class Robot : public frc::TimedRobot
{
public:
  Robot()
  {
    m_left.SetInverted(true);
    m_robotDrive.SetExpiration(100_ms);
    m_timer.Start();
  }

  void AutonomousInit() override
  {
    m_timer.Reset();
    m_timer.Start();
  }

  void AutonomousPeriodic() override
  {
    // Drive for 2 seconds
    if (m_timer.Get() < 2_s)
    {
      // Drive forwards half speed
      m_robotDrive.ArcadeDrive(0.5, 0.0);
    }
    else
    {
      // Stop robot
      m_robotDrive.ArcadeDrive(0.0, 0.0);
    }
  }

  void TeleopInit() override 
  {
    m_runIntake = false;
  }

  void TeleopPeriodic() override
  {
    // Drive with arcade style (use right stick)
    m_robotDrive.ArcadeDrive(-m_stick.GetY(), m_stick.GetX());
    // Intake only activited with a trigger press
    if (m_stick.GetTriggerPressed())
    {
      m_runIntake = !m_runIntake;
    }
    if (m_runIntake)
    {
    // Throttle is connected the slider on the controller
      m_intakeDrive.Set(m_stick.GetThrottle());
    } else {
      m_intakeDrive.Set(0);
    }
  }

private:
  frc::PWMSparkMax m_rightA{0}; // The number is the PWM channel on the rio.
  frc::PWMSparkMax m_rightB{2};
  frc::PWMSparkMax m_leftA{1};
  frc::PWMSparkMax m_leftB{3};
  frc::MotorControllerGroup m_right{m_rightA, m_rightB};
  frc::MotorControllerGroup m_left{m_leftA, m_leftB};
  frc::DifferentialDrive m_robotDrive{m_left, m_right};
  //Sample intake motor controller
  frc::PWMSparkMax m_intakeDrive{4};
  bool m_runIntake;

  frc::Joystick m_stick{0};
  frc::Timer m_timer;
};

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
