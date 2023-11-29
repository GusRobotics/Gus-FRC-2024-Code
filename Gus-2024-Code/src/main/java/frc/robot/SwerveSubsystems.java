package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class SwerveSubsystem extends SubsystemBase {
    private final SwerveModule blue = new SwerveModule(
            constant.blueDrive,
            constant.blueSteer,
            constant.kBlueDriveEncoderReversed,
            constant.kBlueTurningEncoderReversed,
            constant.kBlueDriveAbsoluteEncoderPort,
            constant.kBlueDriveAbsoluteEncoderOffsetRad,
            constant.kBlueDriveAbsoluteEncoderReversed);

    private final SwerveModule orange = new SwerveModule(
            constant.orangeDrive,
            constant.orangeSteer,
            constant.kOrangeDriveEncoderReversed,
            constant.kOrangeTurningEncoderReversed,
            constant.kOrangeDriveAbsoluteEncoderPort,
            constant.kOrangeDriveAbsoluteEncoderOffsetRad,
            constant.kOrangeDriveAbsoluteEncoderReversed);

    private final SwerveModule green = new SwerveModule(
            constant.greenDrive,
            constant.greenSteer,
            constant.kGreenDriveEncoderReversed,
            constant.kGreenTurningEncoderReversed,
            constant.kGreenDriveAbsoluteEncoderPort,
            constant.kGreenDriveAbsoluteEncoderOffsetRad,
            constant.kGreenDriveAbsoluteEncoderReversed);

    private final SwerveModule red = new SwerveModule(
            constant.redDrive,
            constant.redSteer,
            constant.kRedDriveEncoderReversed,
            constant.kRedTurningEncoderReversed,
            constant.kRedDriveAbsoluteEncoderPort,
            constant.kRedDriveAbsoluteEncoderOffsetRad,
            constant.kRedDriveAbsoluteEncoderReversed);

    private final AHRS gyro = new AHRS(SPI.Port.kMXP);
    private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(DriveConstants.kDriveKinematics,
            new Rotation2d(0));

    public SwerveSubsystem() {
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();
    }

    public void zeroHeading() {
        gyro.reset();
    }

    public double getHeading() {
        return Math.IEEEremainder(gyro.getAngle(), 360);
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        odometer.resetPosition(pose, getRotation2d());
    }

    @Override
    public void periodic() {
        odometer.update(getRotation2d(), blue.getState(), orange.getState(), green.getState(),
                red.getState());
        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
    }

    public void stopModules() {
        blue.stop();
        orange.stop();
        green.stop();
        red.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.normalizeWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        blue.setDesiredState(desiredStates[0]);
        orange.setDesiredState(desiredStates[1]);
        green.setDesiredState(desiredStates[2]);
        red.setDesiredState(desiredStates[3]);
    }
}