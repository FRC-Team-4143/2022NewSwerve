// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/DiffSwerveModule.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/geometry/Rotation2d.h>
#include <wpi/numbers>

#include "Constants.h"

#include <frc/Preferences.h>

DiffSwerveModule::DiffSwerveModule(int driveMotorChannel, int turningMotorChannel,
                           int steeringEncoderID, std::string configName)
    : m_driveMotor(driveMotorChannel),
      m_turningMotor(turningMotorChannel),
      m_steeringEncoder(steeringEncoderID) {
  _configName = configName; //Stores the module name
  m_turningPIDController.EnableContinuousInput(units::radian_t(-wpi::numbers::pi),
                                               units::radian_t(wpi::numbers::pi));
}


frc::SwerveModuleState DiffSwerveModule::GetState() {
  return {units::meters_per_second_t{GetDriveMotorSpeed()},
          frc::Rotation2d(units::radian_t(m_steeringEncoder.GetVelocity() * (wpi::numbers::pi / 180.0)))};
}

// =========================Wheel Offsets=======================================

void DiffSwerveModule::SetWheelOffset() {
	auto steerPosition = m_steeringEncoder.GetPosition();
	frc::Preferences::SetDouble(_configName, steerPosition);
  _offset = steerPosition;
}

void DiffSwerveModule::LoadWheelOffset() {
	auto steerPosition = frc::Preferences::GetDouble(_configName);
	_offset = steerPosition;
}

// ================================================================

float DiffSwerveModule::GetDriveMotorSpeed(){
    return ((m_driveMotor.GetSelectedSensorVelocity() - m_turningMotor.GetSelectedSensorVelocity()) / 2.0) 
        * (10.0 / 2048) /*Revs per second*/ * ((10 / 88.0) * (54 / 14.0) * (1 / 3.0)) /*Gear Ratios*/ * (4.5 * 0.0254 * wpi::numbers::pi) /*Axle Revs per Second*/;
}

float DiffSwerveModule::GetSteeringPosition(){
    return ((m_steeringEncoder.GetPosition() - _offset) * (wpi::numbers::pi / 180.0));
}

// =============================

void DiffSwerveModule::SetDesiredState(
    const frc::SwerveModuleState& referenceState) {
  // Optimize the reference state to avoid spinning further than 90 degrees
  const auto state = frc::SwerveModuleState::Optimize(
      referenceState, units::radian_t(GetSteeringPosition()));

  // Calculate the drive output from the drive PID controller.
  const auto driveOutput = m_drivePIDController.Calculate(
      GetDriveMotorSpeed(), state.speed.to<double>());

  // Calculate the turning motor output from the turning PID controller.
  auto turnOutput = m_turningPIDController.Calculate(
      units::radian_t(GetSteeringPosition()), state.angle.Radians());

  // Set the motor outputs.

  //-------------Main Motor Ouput------------------------//

  m_driveMotor.Set(ControlMode::PercentOutput, (driveOutput + turnOutput) * 1.0);
  m_turningMotor.Set(ControlMode::PercentOutput, (-driveOutput + turnOutput) * 1.0);

  //basic motor test  
    // m_driveMotor.Set(ControlMode::PercentOutput, 0.1); 
    // m_turningMotor.Set(ControlMode::PercentOutput, 0.1);

  //-------------SmartDashboard Printing--------------------//
  frc::SmartDashboard::PutNumber("Drive Output", driveOutput);
  frc::SmartDashboard::PutNumber("Turn Output", turnOutput);

  frc::SmartDashboard::PutNumber("Drive Speed", GetDriveMotorSpeed());
  frc::SmartDashboard::PutNumber("Turn Radians", GetSteeringPosition());
}

void DiffSwerveModule::ResetEncoders() {
}
