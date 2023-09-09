// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DrivingSubsystem extends SubsystemBase {
  Spark m_frontLeftTurningMotor = new Spark(Ports.Swerve.FRONT_LEFT_TURNING_MOTOR);
  Spark m_frontLeftDrivingMotor = new Spark(Ports.Swerve.FRONT_LEFT_DRIVING_MOTOR);
  ... // TODO

  SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
    SwerveModuleLocationsMetres.FRONT_LEFT,
    SwerveModuleLocationsMetres.FRONT_RIGHT,
    SwerveModuleLocationsMetres.REAR_LEFT,
    SwerveModuleLocationsMetres.REAR_RIGHT
  );

  public DrivingSubsystem() {}

  @Override
  public void periodic() {}

  public void setState(
    SwerveModuleState frontLeft,
    SwerveModuleState frontRight,
    SwerveModuleState rearLeft,
    SwerveModuleState rearRight
  ) {
    SwerveModuleState optFrontLeft = SwerveModuleState.optimize(
      frontLeft,
      new Rotation2d(m_turningEncoder.getDistance()) // TODO
    );
    ... // TODO
  }

  private static void setModuleState(
    ?? m_turningMotor,
    ?? m_drivingMotor,
    SwerveModuleState state
  ) {
    // Prevent any rotation of more than 90 degrees
    SwerveModuleState optimalState = SwerveModuleState.optimize(
      state,
      new Rotation2d(m_turningMotor.?????getencoder???.getDistance())
    );

    
  }
}
