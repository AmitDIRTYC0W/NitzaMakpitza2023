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
import frc.robot.Constants.Swerve;
import frc.robot.Constants.Swerve.Ports.SwervePorts;
import frc.robot.Constants.Swerve.MechanicalConstants.SwerveMechanicalConstants.EncoderOffsetDegrees;
import frc.robot.Constants.Swerve.MechanicalConstants.SwerveMechanicalConstants.ModuleLocationsMetres;

public class DrivetrainSubsystem extends SubsystemBase {
  private final WPI_TalonFX flDrivingMotor = new WPI_TalonFX(SwervePorts.FRONT_LEFT_DRIVING_MOTOR);
  private final CANSparkMax flSteeringMotor = new CANSparkMax(SwervePorts.FRONT_LEFT_STEERING_MOTOR, MotorType.kBrushless);
  private final WPI_CANCoder flSteeringEncoder = new WPI_CANCoder(SwervePorts.FRONT_LEFT_STEERING_MOTOR);
 
  private final WPI_TalonFX frDrivingMotor = new WPI_TalonFX(SwervePorts.FRONT_RIGHT_DRIVING_MOTOR);
  private final CANSparkMax frSteeringMotor = new CANSparkMax(SwervePorts.FRONT_RIGHT_STEERING_MOTOR, MotorType.kBrushless);
  private final WPI_CANCoder frSteeringEncoder = new WPI_CANCoder(SwervePorts.FRONT_RIGHT_STEERING_MOTOR);
 
  private final WPI_TalonFX rlDrivingMotor = new WPI_TalonFX(SwervePorts.REAR_LEFT_DRIVING_MOTOR);
  private final CANSparkMax rlSteeringMotor = new CANSparkMax(SwervePorts.REAR_LEFT_STEERING_MOTOR, MotorType.kBrushless);
  private final WPI_CANCoder rlSteeringEncoder = new WPI_CANCoder(SwervePorts.REAR_LEFT_STEERING_MOTOR);
 
  private final WPI_TalonFX rrDrivingMotor = new WPI_TalonFX(SwervePorts.REAR_RIGHT_DRIVING_MOTOR);
  private final CANSparkMax rrSteeringMotor = new CANSparkMax(SwervePorts.REAR_RIGHT_STEERING_MOTOR, MotorType.kBrushless);
  private final WPI_CANCoder rrSteeringEncoder = new WPI_CANCoder(SwervePorts.REAR_RIGHT_STEERING_MOTOR);
 
  private final SwerveModule flModule = new SwerveModule(
    0,
    Swerve.Mod0.constants
  );
  private final SwerveModule frModule = new SwerveModule(
    1,
    Swerve.Mod1.constants
  );
  private final SwerveModule rlModule = new SwerveModule(
    2,
    Swerve.Mod2.constants
  );
  private final SwerveModule rrModule = new SwerveModule(
    3,
    Swerve.Mod3.constants
  );

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
    ModuleLocationsMetres.FRONT_LEFT,
    ModuleLocationsMetres.FRONT_RIGHT,
    ModuleLocationsMetres.REAR_LEFT,
    ModuleLocationsMetres.REAR_RIGHT
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
