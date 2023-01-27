// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>

#include <frc/motorcontrol/Victor.h>
#include <frc/ADIS16470_IMU.h>
#include <frc/XboxController.h>

#include <frc/smartdashboard/SmartDashboard.h>

class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void TestInit() override;
  void TestPeriodic() override;
  void SimulationInit() override;
  void SimulationPeriodic() override;

  double MathMax(double a, double b);

 private:
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;

  // Motors
  frc::Victor m_topLeft{1};
  frc::Victor m_topRight{0};
  frc::Victor m_bottomLeft{4};
  frc::Victor m_bottomRight{3};

  // Controller
  frc::XboxController m_xBox{0};

  // IMU / GYRO
  frc::ADIS16470_IMU m_imu;
};
