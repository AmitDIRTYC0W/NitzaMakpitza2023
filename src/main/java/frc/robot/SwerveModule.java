package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import frc.lib.math.Conversions;
import frc.lib.util.CANSparkMaxUtil;
import frc.lib.util.CTREModuleState;
import frc.lib.util.SwerveModuleConstants;
import frc.lib.util.CANSparkMaxUtil.Usage;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
<<<<<<< HEAD
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class SwerveModule {
    public int moduleNumber;
    private Rotation2d angleOffset;
    private Rotation2d lastAngle;
=======

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ControlConstants;
import frc.robot.Constants.MechanicalConstants;
import frc.robot.Constants.MechanicalConstants.SwerveMechanicalConstants;

public class SwerveModule {
  private final WPI_TalonFX drivingMotor;
  private final CANSparkMax steeringMotor;
  private final RelativeEncoder steeringEncoder;
>>>>>>> 38e796776471ec782b914678799b273f1171ef67

    private RelativeEncoder mIntegratedAngleEncoder;
    private SparkMaxPIDController angleController;
    private CANSparkMax mAngleMotor;
    private TalonFX mDriveMotor;
    private CANCoder angleEncoder;

<<<<<<< HEAD
    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants){
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;
        
        /* Angle Encoder Config */
        angleEncoder = new CANCoder(moduleConstants.cancoderID);
        configAngleEncoder();
=======
  public SwerveModule(
    WPI_TalonFX drivingMotor,
    CANSparkMax steeringMotor,
    WPI_CANCoder absoluteSteeringEncoder,
    double absoluteSteeringEncoderOffsetDegrees
  ) {
    this.drivingMotor = drivingMotor;
    this.steeringMotor = steeringMotor;
    this.steeringEncoder = steeringMotor.getEncoder();

    drivingMotor.configFactoryDefault();
    steeringMotor.restoreFactoryDefaults();
    absoluteSteeringEncoder.configFactoryDefault();
>>>>>>> 38e796776471ec782b914678799b273f1171ef67

        /* Angle Motor Config */
        mAngleMotor = new CANSparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
        mIntegratedAngleEncoder = mAngleMotor.getEncoder();
        angleController = mAngleMotor.getPIDController();
        configAngleMotor();

<<<<<<< HEAD
        /* Drive Motor Config */
        mDriveMotor = new TalonFX(moduleConstants.driveMotorID);
        configDriveMotor();

        lastAngle = getState().angle;
=======
    // Only use absoluteSteeringEncoder to reset the relative encoder
    steeringEncoder.setPosition(
      absoluteSteeringEncoder.getAbsolutePosition() - absoluteSteeringEncoderOffsetDegrees
    );
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

    double drivingWheelRPS = optimalState.speedMetersPerSecond / SwerveMechanicalConstants.WHEEL_CIRCUMFERENCE;
    double drivingMotorRPS = drivingWheelRPS * SwerveMechanicalConstants.DRIVING_GEAR_RATIO;
    double drivingMotorFXSpeed = drivingMotorRPS * 10 * Constants.FALCON_SENSOR_TICKS_PER_REV;
    drivingMotor.set(
      ControlMode.Velocity,
      drivingMotorFXSpeed,
      DemandType.ArbitraryFeedForward,
      drivingControllerFeedforward.calculate(drivingMotorFXSpeed)
    );

    // Prevent rotation if speed is insufficient to prevent jittering
    if (optimalState.speedMetersPerSecond < ControlConstants.SWERVE_IN_PLACE_DRIVE_MPS) {
      double steeringMotorRotations = optimalState.angle.getRotations() / SwerveMechanicalConstants.DRIVING_GEAR_RATIO;
      steeringController.setSetpoint(Units.rotationsToDegrees(steeringMotorRotations));      
>>>>>>> 38e796776471ec782b914678799b273f1171ef67
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
        /* This is a custom optimize function, since default WPILib optimize assumes continuous controller which CTRE and Rev onboard is not */
        desiredState = CTREModuleState.optimize(desiredState, getState().angle); 
        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
        if(isOpenLoop){
            double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
            mDriveMotor.set(ControlMode.PercentOutput, percentOutput);
        }
        else {
            double velocity = Conversions.MPSToFalcon(desiredState.speedMetersPerSecond, Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio);
            mDriveMotor.set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward, feedforward.calculate(desiredState.speedMetersPerSecond));
        }
    }

    private void setAngle(SwerveModuleState desiredState){
        Rotation2d angle =
        (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01))
            ? lastAngle
            : desiredState.angle;        angleController.setReference(angle.getDegrees(), ControlType.kPosition);
        
        lastAngle = angle;
    }

    private Rotation2d getAngle(){
        return Rotation2d.fromDegrees(mIntegratedAngleEncoder.getPosition());
    }

    public Rotation2d getCanCoder(){
        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
    }

    public void resetToAbsolute(){
        double absolutePosition = getCanCoder().getDegrees() - angleOffset.getDegrees();
        mIntegratedAngleEncoder.setPosition(absolutePosition);
    }

    private void configAngleEncoder(){        
        angleEncoder.configFactoryDefault();
        angleEncoder.configAllSettings(Robot.ctreConfigs.swerveCanCoderConfig);
    }

    private void configAngleMotor(){
        mAngleMotor.restoreFactoryDefaults();
        CANSparkMaxUtil.setCANSparkMaxBusUsage(mAngleMotor, Usage.kPositionOnly);
        mAngleMotor.setSmartCurrentLimit(Constants.Swerve.angleContinuousCurrentLimit);
        mAngleMotor.setInverted(Constants.Swerve.angleInvert);
        mAngleMotor.setIdleMode(Constants.Swerve.angleNeutralMode);
        mIntegratedAngleEncoder.setPositionConversionFactor(Constants.Swerve.angleConversionFactor);
        angleController.setP(Constants.Swerve.angleKP);
        angleController.setI(Constants.Swerve.angleKI);
        angleController.setD(Constants.Swerve.angleKD);
        angleController.setFF(Constants.Swerve.angleKFF);
        mAngleMotor.enableVoltageCompensation(Constants.Swerve.voltageComp);
        mAngleMotor.burnFlash();
        resetToAbsolute();
    }

    private void configDriveMotor(){        
        mDriveMotor.configFactoryDefault();
        mDriveMotor.setInverted(Constants.Swerve.driveMotorInvert);
        mDriveMotor.configAllSettings(Robot.ctreConfigs.swerveDriveFXConfig);
        mDriveMotor.setNeutralMode(Constants.Swerve.driveNeutralMode);
        mDriveMotor.setSelectedSensorPosition(0);
    }

    private void conifgInverted() {
        switch (this.moduleNumber) {
            case 1:
                mDriveMotor.setInverted(!Constants.Swerve.driveMotorInvert);
                break;
            case 0:
                mDriveMotor.setInverted(!Constants.Swerve.driveMotorInvert);
                break;
            default:
                break;
        }
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(
            Conversions.falconToMPS(mDriveMotor.getSelectedSensorVelocity(), Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio), 
            getAngle()
        ); 
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            Conversions.falconToMeters(mDriveMotor.getSelectedSensorPosition(), Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio), 
            getAngle()
        );
    }

    public void setInverted(boolean isInverted) {
        mAngleMotor.setInverted(isInverted);
    }
}