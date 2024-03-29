package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;


public final class Constants {

    public static final class ModuleConstants {
        public static final double kWheelDiameterMeters = 4*2.54/100;
        public static final double kDriveMotorGearRatio = 1 / 5.95;
        public static final double kTurningMotorGearRatio = 1 /19.61112;    //19.6115
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
        public static final double kPTurning = 0.75;
        public static final double kITurning = 0;
        public static final double kDTurning = 0.013;
    }

    public static final class DriveConstants {
        

        public static final double kTrackWidth = Units.inchesToMeters(21.4);
        // Distance between right and left wheels
        public static final double kWheelBase = Units.inchesToMeters(21.4);
        // Distance between front and back wheels
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2));

        public static final int kFrontLeftDriveMotorPort = 5;
        public static final int kBackLeftDriveMotorPort = 6;
        public static final int kFrontRightDriveMotorPort = 7;
        public static final int kBackRightDriveMotorPort = 8;

        public static final int kFrontLeftTurningMotorPort = 1;
        public static final int kBackLeftTurningMotorPort = 2;
        public static final int kFrontRightTurningMotorPort = 3;
        public static final int kBackRightTurningMotorPort = 4;

        public static final boolean kFrontLeftTurningEncoderReversed = false;
        public static final boolean kBackLeftTurningEncoderReversed = false;
        public static final boolean kFrontRightTurningEncoderReversed = false;
        public static final boolean kBackRightTurningEncoderReversed = false;

        public static final boolean kFrontLeftDriveEncoderReversed = true;
        public static final boolean kBackLeftDriveEncoderReversed = true;
        public static final boolean kFrontRightDriveEncoderReversed = false;
        public static final boolean kBackRightDriveEncoderReversed = false;

        public static final int kFrontLeftDriveAbsoluteEncoderPort = 9;
        public static final int kBackLeftDriveAbsoluteEncoderPort = 10;
        public static final int kFrontRightDriveAbsoluteEncoderPort = 11;
        public static final int kBackRightDriveAbsoluteEncoderPort = 12;

        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

        public static final double kFrontLeftAbsoluteEncoderOffsetRad = 2.268757585282863;
        public static final double kBackLeftAbsoluteEncoderOffsetRad = 0.486271909759748;
        public static final double kFrontRightAbsoluteEncoderOffsetRad = 5.755495916146925;
        public static final double kBackRightAbsoluteEncoderOffsetRad = 3.239767424014474; 

        public static final double kPhysicalMaxSpeedMetersPerSecond = 5;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 4.2 * 2 * Math.PI;//最大旋轉速度

        public static final double kTeleDriveMaxSpeedMetersPerSecond = (kPhysicalMaxSpeedMetersPerSecond / 4)*3.2;//最大加速度3.75
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = //
                kPhysicalMaxAngularSpeedRadiansPerSecond / 4;
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 4;
        public static final double kMaxAngularSpeedRadiansPerSecond = //
                DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 10;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;
        public static final double kPXController = 0.083;
        public static final double kPYController = 0.083;
        public static final double kPThetaController = 0.083;



        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
                new TrapezoidProfile.Constraints(
                        kMaxAngularSpeedRadiansPerSecond,
                        kMaxAngularAccelerationRadiansPerSecondSquared);
        public static final String kAutoDriveSpeed = null;
        public static Object kAutoDriveDistanceInches;
    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kControlPort = 1;

        public static final int kDriverYAxis = 1;
        public static final int kDriverXAxis = 0;
        public static final int kDriverRotAxis = 4;
        public static final int kDriverFieldOrientedButtonIdx = 1;

        public static final double kDeadband = 0.06;
    }

    public static class ShooterConstants{
        public static final int kRightShooterMotorPort = 13;
        public static final int kLeftShooterMotorPort = 14;
      }

    public static class IntakeConstants{
        public static final int kIntakeMotorPort = 17;
        public static final int kIntakeCtrlMotorPort = 18;
      }

    public static class ElvatorConstants{
        public static final int kElvatorRightMotorID = 15;
        public static final int kElvatorLeftMotorID = 16;
    }

} 