// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ControlConstants;
import frc.robot.Constants.MechanicalConstants;

public class SwerveModule {
  private final WPI_TalonFX drivingMotor;
  private final CANSparkMax steeringMotor;
  private final WPI_CANCoder steeringEncoder;

  private final SimpleMotorFeedforward drivingControllerFeedforward = new SimpleMotorFeedforward(
    ControlConstants.SWERVE_DRIVING_KS,
    ControlConstants.SWERVE_DRIVING_KV,
    ControlConstants.SWERVE_DRIVING_KA
  );
  private final PIDController steeringController = new PIDController(
    ControlConstants.SWERVE_DRIVING_KP,
    ControlConstants.SWERVE_DRIVING_KI,
    ControlConstants.SWERVE_DRIVING_KD
  );

  public SwerveModule(WPI_TalonFX drivingMotor, CANSparkMax steeringMotor, WPI_CANCoder steeringEncoder) {
    this.drivingMotor = drivingMotor;
    this.steeringMotor = steeringMotor;
    this.steeringEncoder = steeringEncoder;

    drivingMotor.configFactoryDefault();
    steeringMotor.restoreFactoryDefaults();
    steeringEncoder.configFactoryDefault();

    SlotConfiguration drivingMotorPID = new SlotConfiguration();
    drivingMotorPID.kP = ControlConstants.SWERVE_DRIVING_KP;
    drivingMotorPID.kI = ControlConstants.SWERVE_DRIVING_KI;
    drivingMotorPID.kD = ControlConstants.SWERVE_DRIVING_KD;
    drivingMotorPID.kF = ControlConstants.SWERVE_DRIVING_KF;
    drivingMotor.configureSlot(drivingMotorPID);

    steeringEncoder.setPosition(0);
  }

  public void periodic() {
    steeringMotor.set(steeringController.calculate(steeringEncoder.getPosition()));
  }

  public void setState(SwerveModuleState state) {
    // Prevent any rotation of more than 90 degrees
    SwerveModuleState optimalState = SwerveModuleState.optimize(
      state,
      Rotation2d.fromDegrees(steeringEncoder.getPosition())
    );

    double drivingWheelRPS = optimalState.speedMetersPerSecond / MechanicalConstants.SWERVE_WHEEL_CIRCUMFERENCE;
    double drivingMotorRPS = drivingWheelRPS * MechanicalConstants.SWERVE_DRIVING_GEAR_RATIO;
    double drivingMotorFXSpeed = drivingMotorRPS * 10 * Constants.FALCON_SENSOR_TICKS_PER_REV;
    drivingMotor.set(
      ControlMode.Velocity,
      drivingMotorFXSpeed,
      DemandType.ArbitraryFeedForward,
      drivingControllerFeedforward.calculate(drivingMotorFXSpeed)
    );

    // Prevent rotation if speed is insufficient to prevent jittering
    if (optimalState.speedMetersPerSecond < ControlConstants.SWERVE_IN_PLACE_DRIVE_MPS) {
      double steeringMotorRotations = optimalState.angle.getRotations() / MechanicalConstants.SWERVE_STEERING_GEAR_RATIO;
      steeringController.setSetpoint(Units.rotationsToDegrees(steeringMotorRotations));      
    }
  }

  public void stopMotor() {
    drivingMotor.stopMotor();
    steeringMotor.stopMotor();
  }
}
