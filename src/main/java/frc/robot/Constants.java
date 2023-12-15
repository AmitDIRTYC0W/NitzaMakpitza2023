package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSFalconSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
<<<<<<< HEAD
    public static final double stickDeadband = 0.1;

    public static final class Swerve {
        public static final int pigeonID = 15;
        public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

        public static final COTSFalconSwerveConstants chosenModule =  //TODO: This must be tuned to specific robot
            COTSFalconSwerveConstants.SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L2);

        /* Drivetrain Constants */
        public static final double trackWidth = 0.565;
        public static final double wheelBase = 0.615;
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final boolean angleMotorInvert = chosenModule.angleMotorInvert;
        public static final boolean driveMotorInvert = false;

        /* Angle Encoder Invert */
        public static final boolean canCoderInvert = chosenModule.canCoderInvert;

        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 25;
        public static final int anglePeakCurrentLimit = 40;
        public static final double anglePeakCurrentDuration = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveContinuousCurrentLimit = 35;
        public static final int drivePeakCurrentLimit = 60;
        public static final double drivePeakCurrentDuration = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKFF = 0;
        public static final double angleKP = 0.01;
        public static final double angleKI = 0;
        public static final double angleKD = 0;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.001;
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values 
         * Divide SYSID values by 12 to convert from volts to percent output for CTRE */
        public static final double driveKS = 0; //(0.32 / 12); //TODO: This must be tuned to specific robot
        public static final double driveKV = 0; //(1.51 / 12);
        public static final double driveKA =0; // (0.27 / 12);

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 4.5; //TODO: This must be tuned to specific robot
        /** Radians per Second */
        public static final double maxAngularVelocity = 10.0; //TODO: This must be tuned to specific robot

        /* Neutral Modes */
        public static final IdleMode angleNeutralMode = IdleMode.kCoast;
        public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 9;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(181.56);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 2;
            public static final int canCoderID = 3;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(100.45);//281.51);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 10;
            public static final int angleMotorID = 11;
            public static final int canCoderID = 12;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(194.23);//196.78);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 4;
            public static final int angleMotorID = 5;
            public static final int canCoderID = 6;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(113.46);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        public static boolean angleInvert = true;
        public static double voltageComp = 12.0;
        public static final double angleConversionFactor = 360.0 / angleGearRatio;
=======
    public static class MechanicalConstants {
        public static class SwerveMechanicalConstants {
            public static class ModuleLocationsMetres {
                /**
                 * The location of each swerve module relative to the robot's centre
                 */

                public static final Translation2d FRONT_LEFT = new Translation2d(-0.2975, 0.3325);
                public static final Translation2d FRONT_RIGHT = new Translation2d(0.2975, 0.3225);
                public static final Translation2d REAR_LEFT = new Translation2d(-0.2975, -0.3225);
                public static final Translation2d REAR_RIGHT = new Translation2d(0.2975, -0.3225);
            }

            public static class EncoderOffsetDegrees {
                public static final double FRONT_LEFT = 1.56;
                public static final double FRONT_RIGHT = 100.45;
                public static final double REAR_LEFT = 194.23;
                public static final double REAR_RIGHT = 293.46;
            }

            public static final double WHEEL_DIAMETER = Units.inchesToMeters(4);
            public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * 2 * Math.PI;

            /**
             * The gearing-ratio between the driving motor and the driving wheel using the L2 ratio of
             * the MK4i modules
             */
            public static final double DRIVING_GEAR_RATIO = 6.75;

            /**
             * The ratio between steering motor rotation and the steering itself 
             */
            public static final double STEERING_GEAR_RATIO = 150 / 7;
        }
>>>>>>> 38e796776471ec782b914678799b273f1171ef67
    }

    public static final class AutoConstants { //TODO: The below constants are used in the example auto, and must be tuned to specific robot
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
<<<<<<< HEAD
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;
    
        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
=======
        public static final double SWERVE_STEERING_KP = 0;
        public static final double SWERVE_STEERING_KI = 0;
        public static final double SWERVE_STEERING_KD = 0;

        /**
         * We avoid steering the swerve wheels below a certain driving speed, for in-place turning
         * causes them to jitter. Thus, we hereby define the maximum driving speed that is
         * considered 'in-place'.
         */
        public static final double SWERVE_IN_PLACE_DRIVE_MPS = 0.1;
    }

    public static class Ports {
        public static class Swerve {
            public static final int FRONT_LEFT_DRIVING_MOTOR = 7;
            public static final int FRONT_LEFT_STEERING_MOTOR = 8;
            public static final int FRONT_LEFT_STEERING_ENCODER = 9;

            public static final int FRONT_RIGHT_DRIVING_MOTOR = 1;
            public static final int FRONT_RIGHT_STEERING_MOTOR = 2;
            public static final int FRONT_RIGHT_STEERING_ENCODER = 3;

            public static final int REAR_LEFT_DRIVING_MOTOR = 10;
            public static final int REAR_LEFT_STEERING_MOTOR = 11;
            public static final int REAR_LEFT_STEERING_ENCODER = 12;

            public static final int REAR_RIGHT_DRIVING_MOTOR = 4;
            public static final int REAR_RIGHT_STEERING_MOTOR = 5;
            public static final int REAR_RIGHT_STEERING_ENCODER = 6;
        }
>>>>>>> 38e796776471ec782b914678799b273f1171ef67
    }
}
