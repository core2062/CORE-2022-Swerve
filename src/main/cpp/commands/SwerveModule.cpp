// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/SwerveModule.h"

#include <frc/geometry/Rotation2d.h>
#include <wpi/numbers>

#include "Constants.h"

SwerveModule::SwerveModule(int driveMotorChannel, int turningMotorChannel,
                           const int turningEncoderPorts,
                           bool driveEncoderReversed,
                           bool turningEncoderReversed)
    : m_driveMotor(driveMotorChannel),
      m_turningMotor(driveMotorChannel),
      m_turningEncoder(turningEncoderPorts),
      m_reverseDriveEncoder(driveEncoderReversed),
      m_reverseTurningEncoder(turningEncoderReversed) {
  // Set the distance per pulse for the drive encoder. We can simply use the
  // distance traveled for one rotation of the wheel divided by the encoder
  // resolution.
  // m_driveMotor 
  m_driveMotor.SetSelectedSensorPosition(0,0,50);
  /*
  m_driveEncoder.SetDistancePerPulse(
      ModuleConstants::kDriveEncoderDistancePerPulse);
*/
  // Set the distance (in this case, angle) per pulse for the turning encoder.
  // This is the the angle through an entire rotation (2 * wpi::numbers::pi)
  // divided by the encoder resolution.  
  /*
  m_turningEncoder.SetDistancePerPulse(
      ModuleConstants::kTurningEncoderDistancePerPulse);
*/
  // Limit the PID Controller's input range between -pi and pi and set the input
  // to be continuous.
  m_turningPIDController.EnableContinuousInput(
      units::radian_t(-wpi::numbers::pi), units::radian_t(wpi::numbers::pi));
  m_driveMotor.SetStatusFramePeriod(StatusFrameEnhanced::Status_1_General, 10 , 0);

}

double nativeUnitsToDistanceMeters(double sensorCounts){
    double motorRotations = (double)sensorCounts / ModuleConstants::kEncoderCPR;
    double wheelRotations = motorRotations / 8.14;
    double positionMeters = wheelRotations * (3.1415926535897932384626 * ModuleConstants::kWheelDiameterMeters);
    // std::cout << "Pos Meters: " << positionMeters << "\n";
    return positionMeters;
  }

frc::SwerveModuleState SwerveModule::GetState() {
  return {units::meters_per_second_t{nativeUnitsToDistanceMeters(m_driveMotor.GetSelectedSensorPosition(0))},
          frc::Rotation2d(units::radian_t(m_turningEncoder.GetPosition()))};
}

void SwerveModule::SetDesiredState(
    const frc::SwerveModuleState& referenceState) {
  // Optimize the reference state to avoid spinning further than 90 degrees
  const auto state = frc::SwerveModuleState::Optimize(
      referenceState, units::radian_t(m_turningEncoder.GetPosition()));

    std::cout << "Speed of wheel module: " << state.speed.value() << "\n";
    std::cout << "Encoder: " << m_driveMotor.GetSelectedSensorPosition(0) << "\n";
  
  // Calculate the drive output from the drive PID controller.
  const auto driveOutput = m_drivePIDController.Calculate(
      m_driveMotor.GetSelectedSensorPosition(0), state.speed.value());
    std::cout << "Drive Output: " << driveOutput << "\n";

  // Calculate the turning motor output from the turning PID controller.
  auto turnOutput = m_turningPIDController.Calculate(
      units::radian_t(m_turningEncoder.GetPosition()), state.angle.Radians());
    // std::cout << "Turn Output: " << turnOutput << "\n";

  // Set the motor outputs.
  m_driveMotor.Set(ControlMode::PercentOutput, driveOutput);
  m_turningMotor.Set(ControlMode::PercentOutput, turnOutput);
}

void SwerveModule::ResetEncoders() {
  m_driveMotor.SetSelectedSensorPosition(0.0, 0, 50);
}

