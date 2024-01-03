package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
//import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
//import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;

public class constants {
        // I STOLE THESE CONSTANTS... CONFIRM ACCURACY!
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
        public static final double kDriveMotorGearRatio = 1 / 5.8462;
        public static final double KTurningMotorGearRatio = 1 / 18.0;
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI
                        * kWheelDiameterMeters;
        public static final double kTurningEncoderRot2Rad = KTurningMotorGearRatio * 2 * Math.PI;
        // private static int kDriveEncoderRot2Meter;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
        public static final double kPTurning = 0.25;

        public static final double kTrackWidth = Units.inchesToMeters(21);
        // Distance between right and left wheels
        public static final double kWheelBase = Units.inchesToMeters(25.5);
        // Distance between front and back wheels
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
                        new Translation2d(-kWheelBase / 2, kTrackWidth / 2));

        public static final int blueDrive = 15;
        public static final int blueSteer = 10;
        public static final int greenDrive = 13;
        public static final int greenSteer = 14;
        public static final int orangeDrive = 36;
        public static final int orangeSteer = 7;
        public static final int redDrive = 4;
        public static final int redSteer = 9;

        public static final int kBlueDriveMotorPort = 8;
        public static final int kGreenDriveMotorPort = 2;
        public static final int kOrangeDriveMotorPort = 6;
        public static final int kRedDriveMotorPort = 4;

        public static final int kBlueTurningMotorPort = 7;
        public static final int kGreenTurningMotorPort = 1;
        public static final int kOrangeTurningMotorPort = 5;
        public static final int kRedTurningMotorPort = 3;

        public static final boolean kBlueTurningEncoderReversed = true;
        public static final boolean kGreenTurningEncoderReversed = true;
        public static final boolean kOrangeTurningEncoderReversed = true;
        public static final boolean kRedTurningEncoderReversed = true;

        public static final boolean kBlueDriveEncoderReversed = true;
        public static final boolean kGreenEncoderReversed = true;
        public static final boolean kOrangeDriveEncoderReversed = false;
        public static final boolean kRedDriveEncoderReversed = false;

        public static final int kBlueDriveAbsoluteEncoderPort = 3;
        public static final int kGreenDriveAbsoluteEncoderPort = 5;
        public static final int kOrangeDriveAbsoluteEncoderPort = 1;
        public static final int kRedDriveAbsoluteEncoderPort = 2;

        public static final boolean kBlueDriveAbsoluteEncoderReversed = false;
        public static final boolean kGreenDriveAbsoluteEncoderReversed = false;
        public static final boolean kOrangeDriveAbsoluteEncoderReversed = false;
        public static final boolean kRedDriveAbsoluteEncoderReversed = false;

        public static final double kBlueDriveAbsoluteEncoderOffsetRad = -0.254;
        public static final double kGreenDriveAbsoluteEncoderOffsetRad = -1.252;
        public static final double kOrangeDriveAbsoluteEncoderOffsetRad = -1.816;
        public static final double kRedDriveAbsoluteEncoderOffsetRad = -4.811;

        public static final double kPhysicalMaxSpeedMetersPerSecond = 5;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

        public static final int driveMotorCurrentLimit = 50;

        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 4;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = //
                        kPhysicalMaxAngularSpeedRadiansPerSecond / 4;
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;

        public static final int kDriverControllerPort = 0;

        public static final int kDriverYAxis = 1;
        public static final int kDriverXAxis = 0;
        public static final int kDriverRotAxis = 4;
        public static final int kDriverFieldOrientedButtonIdx = 1;

        public static final double kDeadband = 0.05;

        public static final double OIConstants = 0.05;
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
                        kPhysicalMaxSpeedMetersPerSecond, kPhysicalMaxAngularSpeedRadiansPerSecond);
        TrapezoidProfile.State previousProfiledReference = new TrapezoidProfile.State();
}
