// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/Encoder.h>
#include <frc/motorcontrol/Spark.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <ctre/Phoenix.h>
#include <wpi/numbers>

#include "Constants.h"

class DiffSwerveModule {
  using radians_per_second_squared_t =
      units::compound_unit<units::radians,
                           units::inverse<units::squared<units::second>>>;

 public:
  DiffSwerveModule(int driveMotorChannel, int turningMotorChannel,
               int steeringEncoderID, std::string configName);

  frc::SwerveModuleState GetState();

  void SetWheelOffset();
  void LoadWheelOffset();

  void SetDesiredState(const frc::SwerveModuleState& state);

  void ResetEncoders();

  float GetDriveMotorSpeed();
  float GetSteeringPosition();

 private:
  // We have to use meters here instead of radians due to the fact that
  // ProfiledPIDController's constraints only take in meters per second and
  // meters per second squared.

  //Name of each module
  std::string _configName;

  //stores offset
  double _offset;

  static constexpr units::radians_per_second_t kModuleMaxAngularVelocity =
      units::radians_per_second_t(wpi::numbers::pi);  // radians per second
  static constexpr units::unit_t<radians_per_second_squared_t>
      kModuleMaxAngularAcceleration =
          units::unit_t<radians_per_second_squared_t>(
              wpi::numbers::pi * 2.0);  // radians per second squared

  TalonFX m_driveMotor;
  TalonFX m_turningMotor;

  //frc::Encoder m_driveEncoder;
  //frc::Encoder m_turningEncoder;

  CANCoder m_steeringEncoder;

  bool m_reverseDriveEncoder;
  bool m_reverseTurningEncoder;

  //---------------------------PID------------------------------------

  frc2::PIDController m_drivePIDController{
      ModuleConstants::kPModuleDriveController, 0, 0};
  frc::ProfiledPIDController<units::radians> m_turningPIDController{
      ModuleConstants::kPModuleTurningController,
      0.0,
      0.0,
      {kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration}};
};
