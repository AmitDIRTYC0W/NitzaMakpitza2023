// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SwerveModule;
import frc.robot.Constants.Ports;
import frc.robot.Constants.MechanicalConstants.SwerveModuleLocationsMetres;

public class DrivetrainSubsystem extends SubsystemBase {
  private final WPI_TalonFX flDrivingMotor = new WPI_TalonFX(Ports.Swerve.FRONT_LEFT_DRIVING_MOTOR);
  private final CANSparkMax flSteeringMotor = new CANSparkMax(Ports.Swerve.FRONT_LEFT_STEERING_MOTOR, MotorType.kBrushless);
  private final WPI_CANCoder flSteeringEncoder = new WPI_CANCoder(Ports.Swerve.FRONT_LEFT_STEERING_MOTOR);
 
  private final WPI_TalonFX frDrivingMotor = new WPI_TalonFX(Ports.Swerve.FRONT_RIGHT_DRIVING_MOTOR);
  private final CANSparkMax frSteeringMotor = new CANSparkMax(Ports.Swerve.FRONT_RIGHT_STEERING_MOTOR, MotorType.kBrushless);
  private final WPI_CANCoder frSteeringEncoder = new WPI_CANCoder(Ports.Swerve.FRONT_RIGHT_STEERING_MOTOR);
 
  private final WPI_TalonFX rlDrivingMotor = new WPI_TalonFX(Ports.Swerve.REAR_LEFT_DRIVING_MOTOR);
  private final CANSparkMax rlSteeringMotor = new CANSparkMax(Ports.Swerve.REAR_LEFT_STEERING_MOTOR, MotorType.kBrushless);
  private final WPI_CANCoder rlSteeringEncoder = new WPI_CANCoder(Ports.Swerve.REAR_LEFT_STEERING_MOTOR);
 
  private final WPI_TalonFX rrDrivingMotor = new WPI_TalonFX(Ports.Swerve.REAR_RIGHT_DRIVING_MOTOR);
  private final CANSparkMax rrSteeringMotor = new CANSparkMax(Ports.Swerve.REAR_RIGHT_STEERING_MOTOR, MotorType.kBrushless);
  private final WPI_CANCoder rrSteeringEncoder = new WPI_CANCoder(Ports.Swerve.REAR_RIGHT_STEERING_MOTOR);
 
  private final SwerveModule flModule = new SwerveModule(
    flDrivingMotor,
    flSteeringMotor,
    flSteeringEncoder,
    EncoderOffsetDegrees.FRONT_LEFT
  );
  private final SwerveModule frModule = new SwerveModule(
    frDrivingMotor,
    frSteeringMotor,
    frSteeringEncoder,
    EncoderOffsetDegrees.FRONT_RIGHT
  );
  private final SwerveModule rlModule = new SwerveModule(
    rlDrivingMotor,
    rlSteeringMotor,
    rlSteeringEncoder,
    EncoderOffsetDegrees.REAR_LEFT
  );
  private final SwerveModule rrModule = new SwerveModule(
    rrDrivingMotor,
    rrSteeringMotor,
    rrSteeringEncoder,
    EncoderOffsetDegrees.REAR_RIGHT
  );

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
    SwerveModuleLocationsMetres.FRONT_LEFT,
    SwerveModuleLocationsMetres.FRONT_RIGHT,
    SwerveModuleLocationsMetres.REAR_LEFT,
    SwerveModuleLocationsMetres.REAR_RIGHT
  );

  public DrivetrainSubsystem() {}

  @Override
  public void periodic() {
    flModule.periodic();
    frModule.periodic();
    rlModule.periodic();
    rrModule.periodic();
  }
  
  public void drive(ChassisSpeeds speeds) {
    SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
    flModule.setState(states[0]);
    frModule.setState(states[1]);
    rlModule.setState(states[2]);
    rrModule.setState(states[3]);
  }

  public void stopMotor() {
    flModule.stopMotor();
    frModule.stopMotor();
    rlModule.stopMotor();
    rrModule.stopMotor();
  }
}
