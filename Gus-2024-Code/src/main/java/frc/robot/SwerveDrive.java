package frc.robot;

//import com.kauailabs.navx.frc.AHRS;
//import com.wpilibj.SPI;
//import com.kauailabs.navx.frc.ARHS;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
//import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
//import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//import edu.wpi.first.wpilibj.interfaces;
//import edu.wpi.first.wpilibj.interfaces;
public class SwerveDrive extends SubsystemBase {
    // private final AHRS gyro = new AHRS(SPI.Port.kMXP);
    // private final AHRS gyro = new AHRS(SPI.Port.kMXP); // Uncommented
    private final AHRS gyro = new AHRS(SPI.Port.kMXP);
    //private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(constants.kDriveKinematics,
            //new Rotation2d(0), null);
    private final SwerveModule blue;
    private final SwerveModule red;
    private final SwerveModule green;
    private final SwerveModule orange;

    SwerveModuleState driveStates[] = new SwerveModuleState[4];

    public SwerveDrive() {
        blue = new SwerveModule(
            constants.blueDrive,
            constants.blueSteer,
            constants.kBlueDriveEncoderReversed,
            constants.kBlueTurningEncoderReversed,
            constants.kBlueDriveAbsoluteEncoderPort,
            constants.kBlueDriveAbsoluteEncoderOffsetRad,
            constants.kBlueDriveAbsoluteEncoderReversed,
            false, false);

        orange = new SwerveModule(
            constants.orangeDrive,
            constants.orangeSteer,
            constants.kOrangeDriveEncoderReversed,
            constants.kOrangeTurningEncoderReversed,
            constants.kOrangeDriveAbsoluteEncoderPort,
            constants.kOrangeDriveAbsoluteEncoderOffsetRad,
            constants.kOrangeDriveAbsoluteEncoderReversed,
            true, false);

        green = new SwerveModule(
            constants.greenDrive,
            constants.greenSteer,
            constants.kGreenTurningEncoderReversed,
            constants.kGreenTurningEncoderReversed,
            constants.kGreenDriveAbsoluteEncoderPort,
            constants.kGreenDriveAbsoluteEncoderOffsetRad,
            constants.kGreenDriveAbsoluteEncoderReversed,
            true, false);

        red = new SwerveModule(
            constants.redDrive,
            constants.redSteer,
            constants.kRedDriveEncoderReversed,
            constants.kRedTurningEncoderReversed,
            constants.kRedDriveAbsoluteEncoderPort,
            constants.kRedDriveAbsoluteEncoderOffsetRad,
            constants.kRedDriveAbsoluteEncoderReversed,
            true, false);

        driveStates[0] = blue.getState(); 
        driveStates[1] = orange.getState();
        driveStates[2] = red.getState();
        driveStates[3] = green.getState();
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

    // public Pose2d getPose() {
    //     return odometer.getPoseMeters();
    // }

    public void resetOdometry(Pose2d pose) {
        //odometer.resetPosition(getRotation2d(), null, pose);
    }

    @Override
    public void periodic() {
        //note odometry settings commented out bc of swervedrivestate and swervedriveposition
        //odometer.update(getRotation2d(), driveStates);
        SmartDashboard.putNumber("Robot Heading", getHeading());
        //SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
    }

    public void stopModules() {
        blue.stop();
        orange.stop();
        green.stop();
        red.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, constants.kPhysicalMaxSpeedMetersPerSecond);
        blue.setDesiredState(desiredStates[0]);
        orange.setDesiredState(desiredStates[1]);
        green.setDesiredState(desiredStates[2]);
        red.setDesiredState(desiredStates[3]);
    }
}