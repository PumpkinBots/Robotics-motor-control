// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <fstream>
#include <string>
#include <sstream>

#include <fmt/core.h>
#include <frc/Filesystem.h>
#include <frc/Joystick.h>
#include <frc/XboxController.h>
#include <frc/TimedRobot.h>
#include <frc/Timer.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/SendableChooser.h>


#include "rev/CANSparkMax.h"
#include "IntakeRetractionSubsystem.h"
#include "IntakeSubsystem.h"
#include "LaunchSubsystem.h"
#include "TransportSubsystem.h"
#include "HardwareIDs.h"
#include "RobotVersion.h"
#include "ClimbSubsystem.h"

#include "networktables/NetworkTable.h"
#include "frc/smartdashboard/SmartDashboard.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableValue.h"
#include "wpi/span.h"

#include <units/dimensionless.h>
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
    m_robotDrive.SetExpiration(100_ms);
    m_timer.Start();
    m_testDrives.push_back(&m_leftLeader);
    m_testDrives.push_back(&m_leftFollower);
    m_testDrives.push_back(&m_rightLeader);
    m_testDrives.push_back(&m_rightFollower);
  }

  void RobotInit() override {
    // Initialize drive motors
    m_leftLeader.RestoreFactoryDefaults();
    m_leftFollower.RestoreFactoryDefaults();
    m_rightLeader.RestoreFactoryDefaults();
    m_rightFollower.RestoreFactoryDefaults();
    // Set followers
    m_leftFollower.Follow(m_leftLeader);
    m_rightFollower.Follow(m_rightLeader);
    m_rightLeader.SetInverted(true);

    m_intake.RobotInit();
    m_launch.RobotInit();
    m_transport.RobotInit();
    m_intakeRetraction.RobotInit();
    m_climb.RobotInit();
    

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
    m_intakeRetraction.ModeInit();
    m_climb.ModeInit();

    m_timer.Reset();
    m_timer.Start();
  }

  void AutonomousPeriodic() override
  {
    // automonomous mode objectives.
    // 1) shoot pre-loaded ball in top goal (4 points)
    // 2) drive out of tarmac area (2 points)

    // stage 1.
    // ** calibration: 1) shooter speed 2) robot start location
    // a) enable shooter to calibrated speed.
    // b) enable transport to move ball to shooter.
    // c) (shot occurs)
    // d) disable shooter
    // e) disable shooter
    // f) move to stage 2

    // stage 2.
    // ** calibration: 1) time traveled 2) speed traveled
    // TODO: 2_s and 0.5 speed ends at back wall of field when starting at mid/center
    // a) driver in reverse for 2 seconds.
    
    // Start trying to score immediately.
    // Drive for 2_s after 5_s have passed.
    if (m_timer.Get() < 7_s)
    {
      if (m_timer.Get() < 5_s)
      {
        if (m_timer.Get() < 0.5_s)
        {
          m_robotDrive.ArcadeDrive(0.65, 0.0);
        }
        else
        {
          m_robotDrive.ArcadeDrive(0.0, 0.0);
        }
        // Start spinning the launch wheel immediately and continue for 5_s.
        m_launch.RunAutonomous(true);
        // Start moving the ball into the launcher starting at 3_s.
        if (m_timer.Get() >= 3_s)
        {
          m_transport.RunAutonomous(true);
        }
        else
        {
          m_transport.RunAutonomous(false);
        }
      }
      else // after 5_s stop trying to shoot the cargo and start driving.
      {
        m_launch.RunAutonomous(false);
        m_transport.RunAutonomous(false);
        m_robotDrive.ArcadeDrive(0.65, 0.0);
      }
    }
    else
    {
      // Stop robot
      m_robotDrive.ArcadeDrive(0.0, 0.0);
      m_launch.RunAutonomous(false);
      m_transport.RunAutonomous(false);
    }
    m_intakeRetraction.StopMotor();
    m_climb.StopMotor();
  }

  void TeleopInit() override 
  {
    m_robotDrive.StopMotor();
    m_turnRateLimiter.Reset(0);
    m_intake.ModeInit();
    m_launch.ModeInit();
    m_transport.ModeInit();
    m_intakeRetraction.ModeInit();
    m_climb.ModeInit();
  }

  void TeleopPeriodic() override
  {
    // Y-axis is negative pushed forward, and now the drive forward
    // is also negative. However invert the twist input.
    // m_robotDrive.ArcadeDrive(m_stick.GetY(), m_turnRateLimiter.Calculate(-m_stick.GetTwist()), true);
    m_robotDrive.ArcadeDrive(m_stick.GetY(), (0.5)*-m_stick.GetTwist(), false);

    // Check and run the Subsystems.
    m_intake.RunPeriodic();
    m_launch.RunPeriodic();
    m_transport.RunPeriodic();
    m_intakeRetraction.RunPeriodic();
    m_climb.RunPeriodic();
#if 0
    // Toggle Transport state on button press.
    bool runIntakeRetractionDown = m_xbox.GetLeftBumper();
    if (runIntakeRetractionDown)
    {
    // Throttle is connected the slider on the controller.
    // The throttle axis reads -1.0 when pressed forward.
      m_intakeRetractionDrive.Set(-1.0);
    } else {
      m_intakeRetractionDrive.Set(0);
    }
    frc::SmartDashboard::PutBoolean("Run Intake Retraction Down", runIntakeRetractionDown);

    bool runIntakeRetractionUp = m_xbox.GetRightBumper();
    if (runIntakeRetractionUp)
    {
    // Throttle is connected the slider on the controller.
    // The throttle axis reads -1.0 when pressed forward.
      m_intakeRetractionDrive.Set(1.0);
    } else {
      m_intakeRetractionDrive.Set(0);
    }
    frc::SmartDashboard::PutBoolean("Run Intake Retraction Up", runIntakeRetractionUp);

    bool runClimberRight;
    if (m_xbox.GetYButton())
    {
      runClimberRight = !runClimberRight;
    }
    if (runClimberRight)
    {
      m_climbDrive.Set(m_xbox.GetRightY());
    } else {
      m_climbDrive.Set(0.0);
    }
    frc::SmartDashboard::PutBoolean("Run Left Climber", runClimberRight);

    bool runIntakeXbox;
    if (m_xbox.GetLeftTriggerAxis() > 0){
      runIntakeXbox = true;
    } else {
      runIntakeXbox = false;
    }
    if (runIntakeXbox)
    {
    // Throttle is connected the slider on the controller.
    // The throttle axis reads -1.0 when pressed forward.
      m_intakeDrive.Set(0.8);
    } else {
      m_intakeDrive.Set(0);
    }
    frc::SmartDashboard::PutBoolean("Run Intake", runIntakeXbox);  

    bool runTransportXbox;
    if (m_xbox.GetBButton()){
      runTransportXbox = true;
    } else {
      runTransportXbox = false;
    }
    if (runTransportXbox)
    {
    // Throttle is connected the slider on the controller.
    // The throttle axis reads -1.0 when pressed forward.
      m_transportDrive.Set(0.8);
    } else {
      m_transportDrive.Set(0);
    }
    frc::SmartDashboard::PutBoolean("Run Transport", runTransportXbox);  

    bool runLaunchXbox;
    if (m_xbox.GetRightTriggerAxis() > 0){
      runLaunchXbox = true;
    } else {
      runLaunchXbox = false;
    }
    if (runLaunchXbox)
    {
    // Throttle is connected the slider on the controller.
    // The throttle axis reads -1.0 when pressed forward.
      m_launchDrive.Set(0.687);
    } else {
      m_launchDrive.Set(0);
    }
    frc::SmartDashboard::PutBoolean("Run Launch", runLaunchXbox);  
#endif
  }


  void TestInit() override
  {
    // Disable to drive motors in Test mode so that the robot stays on the bench.
    //m_robotDrive.StopMotor();
    m_intake.ModeInit();
    m_launch.ModeInit();
    m_transport.ModeInit();
    m_intakeRetraction.ModeInit();
    m_climb.ModeInit();

    m_testIndex = 0;
    m_stick.GetRawButtonPressed(testStartButton);
    m_stick.GetRawButtonPressed(testNextButton);
    m_runTest = false;
    frc::SmartDashboard::PutNumber("AAindex", m_testIndex);
    fmt::print("Switched to index {}  device id {}\n", m_testIndex, m_testDrives[m_testIndex]->GetDeviceId());

  }

  void TestPeriodic() override
  {
    // Do not run the drive in Test mode.
    m_intake.RunPeriodic();
    m_launch.RunPeriodic();
    m_transport.RunPeriodic();
    m_intakeRetraction.RunPeriodic();
    m_climb.RunPeriodic();

  frc::SmartDashboard::PutNumber("AAindex", m_testIndex);
  if (m_stick.GetRawButtonPressed(testStartButton)) {
    m_runTest = !m_runTest;
  }
  if (m_runTest) {
    //m_testDrives[m_testIndex]->Set(0.5);
    m_testDrives[m_testIndex]->Set(m_stick.GetThrottle());
  } else {
    m_testDrives[m_testIndex]->Set(0);
    if (m_stick.GetRawButtonPressed(testNextButton)) {
      m_testIndex++;
      if (m_testIndex > 3) { m_testIndex = 0;}
      fmt::print("Switched to index {}  device id {}\n", m_testIndex, m_testDrives[m_testIndex]->GetDeviceId());
    }
  }

  }

private:
  std::string m_buildVersion;
  rev::CANSparkMax m_leftLeader{kDriveLeftLeader, rev::CANSparkMax::MotorType::kBrushed};
  rev::CANSparkMax m_leftFollower{kDriveLeftFollower, rev::CANSparkMax::MotorType::kBrushed};
  rev::CANSparkMax m_rightLeader{kDriveRightLeader, rev::CANSparkMax::MotorType::kBrushed};
  rev::CANSparkMax m_rightFollower{kDriveRightFollower, rev::CANSparkMax::MotorType::kBrushed};
  frc::DifferentialDrive m_robotDrive{m_leftLeader, m_rightLeader};

  frc::Joystick m_stick{0};
  frc::XboxController m_xbox{1};
  frc::SlewRateLimiter<units::scalar> m_turnRateLimiter{1 / 1_s, 0};
  frc::Timer m_timer;

  rev::CANSparkMax m_transportDrive{kTransportDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_launchDrive{kLaunchDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_intakeDrive{kIntakeDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_intakeRetractionDrive{kIntakeRetractionDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_climbDrive{kclimbDeviceID, rev::CANSparkMax::MotorType::kBrushless};

  // Create an IntakeSubsystem to encapsulate the behavior.
  // This object must be created after the objects that it uses.
  // Bind the intake on/off to joystick button 2.
  IntakeSubsystem m_intake{kTransportButton, m_intakeDrive, m_stick};
  // Bind the launch wheel to joystick button 3.
  LaunchSubsystem m_launch{kLaunchButton, m_launchDrive, m_stick};
  // Bind the transport on/off to joystick button 1, the trigger.
  TransportSubsystem m_transport{kTransportButton, m_transportDrive, m_stick};
  // Bind the intake retraction on/off to joytsick button 4.
  IntakeRetractionSubsystem m_intakeRetraction{m_intakeRetractionDrive, m_xbox};
  // Bind the
  ClimbSubsystem m_climb{kClimbButton, m_climbDrive, m_stick};


  // Allow the robot to access the data from the camera. 
  std::shared_ptr<nt::NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
  double targetOffsetAngle_Horizontal = table->GetNumber("tx",0.0);
  double targetOffsetAngle_Vertical = table->GetNumber("ty",0.0);
  double targetArea = table->GetNumber("ta",0.0);
  double targetSkew = table->GetNumber("ts",0.0);

  // Silly independent motor test
  int m_testIndex = 0;
  int testStartButton = 7;
  int testNextButton = 8;
  bool m_runTest = false;
  std::vector<rev::CANSparkMax*> m_testDrives;
};

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
