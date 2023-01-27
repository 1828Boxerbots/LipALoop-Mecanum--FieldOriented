// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <fmt/core.h>

#include <frc/smartdashboard/SmartDashboard.h>


void Robot::RobotInit() {
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
}

/**
 * This function is called every 20 ms, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit() {
  m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString("Auto Selector",
  //     kAutoNameDefault);
  fmt::print("Auto selected: {}\n", m_autoSelected);

  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::AutonomousPeriodic() {
  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::TeleopInit()
{
  m_topLeft.SetInverted(true);
  m_bottomLeft.SetInverted(true);
  m_imu.Reset();
}

void Robot::TeleopPeriodic()
{
  double vertical = 0.0;
  double horizontal = 0.0;
  double pivot = 0.0;

  // Get controller input
  vertical = -m_xBox.GetLeftY() * 0.1;
  horizontal = m_xBox.GetLeftX() * 0.1;
  pivot = m_xBox.GetRightX() * 0.1;

  // Get the angle(in radians) of where the robot wants to go
  // For robot orientation remove the imu
  // For field orientation subtract the vector by the imu Angle as a radian
  double theta = atan2(vertical, horizontal) - ((double)m_imu.GetAngle() * (M_PI / 180.0));

  frc::SmartDashboard::PutNumber("Angle", (double)m_imu.GetAngle());
  frc::SmartDashboard::PutNumber("Theta", theta);
  
  // The power theoretical power the wheels will be going
  double power = hypot(horizontal, vertical);

  // Get the vertical and horizontal vectors
  double vertVector = sin(theta - (M_PI/4));
  double horVector = cos(theta - (M_PI/4));

  double max = MathMax(vertVector, horVector);

  // Set motor power
  // Power * vector will set the correct power for the vectors
  m_topLeft.Set(((power * horVector)/max)+pivot);
  m_topRight.Set(((power * vertVector)/max)-pivot);
  m_bottomLeft.Set(((power * vertVector)/max)+pivot);
  m_bottomRight.Set(((power * horVector)/max)-pivot);
}

double Robot::MathMax(double a, double b)
{
  double returnValue;
  if(fabs(a) > fabs(b))
  {
    returnValue = fabs(a);
  }
  else
  {
    returnValue = fabs(b);
  }

  return returnValue;
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}

void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
