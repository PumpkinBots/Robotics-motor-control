// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <fstream>
#include <string>
#include <sstream>

#include <fmt/core.h>
#include <frc/Filesystem.h>
#include <frc/Joystick.h>
#include <frc/TimedRobot.h>
#include <frc/Timer.h>
#include <frc/motorcontrol/PWMSparkMax.h>
#include <frc/motorcontrol/MotorControllerGroup.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/SendableChooser.h>


#include "rev/CANSparkMax.h"
#include "IntakeSubsystem.h"
#include "LaunchSubsystem.h"
#include "TransportSubsystem.h"
#include "HardwareIDs.h"
#include "RobotVersion.h"

#include "networktables/NetworkTable.h"
#include "frc/smartdashboard/Smartdashboard.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableValue.h"
#include "wpi/span.h"

#include <frc/filter/SlewRateLimiter.h>

#include <frc/filter/SlewRateLimiter.h>

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

  void RobotInit() override {
    m_intake.RobotInit();
    m_launch.RobotInit();
    m_transport.RobotInit();

    // Read the build version from the deploy directory.
    // https://docs.wpilib.org/en/stable/docs/software/advanced-gradlerio/deploy-git-data.html
    std::string deployDir = frc::filesystem::GetDeployDirectory();
    std::ifstream branchFile(deployDir + "/branch.txt");
    std::string branchStr;
    branchFile >> branchStr;  // This should suck up the whole file into the string.
    fmt::print("Branch: {}\n", branchStr.c_str());  // This prints to the console.
    std::ifstream commitFile(deployDir + "/commit.txt");
    std::string commitStr;
    commitFile >> commitStr;  // This should suck up the whole file into the string.
    fmt::print("Commit: {}\n", commitStr.c_str());

    // Format the displayed version using an sstream.
    std::ostringstream buildVersStream;
    buildVersStream << "Branch: " << branchStr << " Commit: " << commitStr;
    m_buildVersion = buildVersStream.str();

    fmt::print("Formated m_buildVersion: |{}|\n",  m_buildVersion);
    frc::SmartDashboard::PutString("Robot Code Version", m_buildVersion);

    std::string gitVersion = GetRobotVersion();
    fmt::print("Version: {}\n", gitVersion);
    frc::SmartDashboard::PutString("Robot Code Git Version", gitVersion);
  }

  void AutonomousInit() override
  {
    m_robotDrive.StopMotor();
    m_intake.ModeInit();
    m_launch.ModeInit();
    m_transport.ModeInit();

    m_timer.Reset();
    m_timer.Start();
  }

  void AutonomousPeriodic() override
  {
    // Drive for 3 seconds
    if (m_timer.Get() < 3_s)
    {
      // Drive backwards half speed
      m_robotDrive.ArcadeDrive(-0.5, 0.0);
    }
    else
    {
      // Stop robot
      m_robotDrive.ArcadeDrive(0.0, 0.0);
    }
  }

  void TeleopInit() override 
  {
    m_robotDrive.StopMotor();
    m_intake.ModeInit();
    m_launch.ModeInit();
    m_transport.ModeInit();
  }

  void TeleopPeriodic() override
  {
    // Y-axis is negative pushed forward, so invert the value.
    m_robotDrive.ArcadeDrive(-m_stick.GetY(), m_stick.GetTwist(), true);

    // Check and run the IntakeSubsystem.
    m_intake.RunPeriodic();
    m_launch.RunPeriodic();
    m_transport.RunPeriodic();

  }

  void TestInit() override
  {
    // Disable to drive motors in Test mode so that the robot stays on the bench.
    m_robotDrive.StopMotor();
    m_intake.ModeInit();
    m_launch.ModeInit();
    m_transport.ModeInit();
  }

  void TestPeriodic() override
  {
    // Do not run the drive in Test mode.
    m_intake.RunPeriodic();
    m_launch.RunPeriodic();
    m_transport.RunPeriodic();

  }

private:
  std::string m_buildVersion;
  frc::PWMSparkMax m_rightA{0}; // The number is the PWM channel on the rio.
  frc::PWMSparkMax m_rightB{2};
  frc::PWMSparkMax m_leftA{1};
  frc::PWMSparkMax m_leftB{3};
  frc::MotorControllerGroup m_right{m_rightA, m_rightB};
  frc::MotorControllerGroup m_left{m_leftA, m_leftB};
  frc::DifferentialDrive m_robotDrive{m_left, m_right};

  frc::Joystick m_stick{0};
  frc::Timer m_timer;

  rev::CANSparkMax m_transportDrive{kTransportDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_launchDrive{kLaunchDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_intakeDrive{kIntakeDeviceID, rev::CANSparkMax::MotorType::kBrushless};

  // Create an IntakeSubsystem to encapsulate the behavior.
  // This object must be created after the objects that it uses.
  // Bind the intake on/off to joystick button 2.
  IntakeSubsystem m_intake{kIntakeButton, m_intakeDrive, m_stick};
  // Bind the launch wheel to joystick button 3.
  LaunchSubsystem m_launch{kLaunchButton, m_launchDrive, m_stick};
  // Bind the transport on/off to joystick button 1, the trigger.
  TransportSubsystem m_transport{kTransportButton, m_transportDrive, m_stick};

  // Allow the robot to access the data from the camera. 
  std::shared_ptr<nt::NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
  double targetOffsetAngle_Horizontal = table->GetNumber("tx",0.0);
  double targetOffsetAngle_Vertical = table->GetNumber("ty",0.0);
  double targetArea = table->GetNumber("ta",0.0);
  double targetSkew = table->GetNumber("ts",0.0);
};

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
