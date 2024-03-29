package frc.robot;
//got rid of that pigeon import bc  for some reason it wasn't getting the values of old class
//import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.core.CorePigeon2;
//import com.ctre.phoenix6.hardware.CANcoder;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.filter.SlewRateLimiter;
//import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDrive extends SubsystemBase {

    private final AHRS gyro = new AHRS(SPI.Port.kMXP);
    // private final SwerveDriveOdometry odometer = new
    // SwerveDriveOdometry(constants.kDriveKinematics,
    // new Rotation2d(0), null);
    private final SwerveModule blue;
    private final SwerveModule red;
    private final SwerveModule green;
    private final SwerveModule orange;
    private SlewRateLimiter xLimiter = new SlewRateLimiter(0.5);
    private SlewRateLimiter yLimiter = new SlewRateLimiter(0.5);
    private SlewRateLimiter turningLimiter = new SlewRateLimiter(0.5);
    private final CorePigeon2 pigeon = new CorePigeon2(constants.kPigeonPort);

    // toggling between SwerveModelState and SwerveModelPosition, attempting to
    // debug odometer
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
                constants.blueDriveInvert,
                constants.blueTurnInvert);

        orange = new SwerveModule(
                constants.orangeDrive,
                constants.orangeSteer,
                constants.kOrangeDriveEncoderReversed,
                constants.kOrangeTurningEncoderReversed,
                constants.kOrangeDriveAbsoluteEncoderPort,
                constants.kOrangeDriveAbsoluteEncoderOffsetRad,
                constants.kOrangeDriveAbsoluteEncoderReversed,
                constants.orangeDriveInvert,
                constants.orangeTurnInvert);

        green = new SwerveModule(
                constants.greenDrive,
                constants.greenSteer,
                constants.kGreenTurningEncoderReversed,
                constants.kGreenTurningEncoderReversed,
                constants.kGreenDriveAbsoluteEncoderPort,
                constants.kGreenDriveAbsoluteEncoderOffsetRad,
                constants.kGreenDriveAbsoluteEncoderReversed,
                constants.greenDriveInvert,
                constants.greenTurnInvert);

        red = new SwerveModule(
                constants.redDrive,
                constants.redSteer,
                constants.kRedDriveEncoderReversed,
                constants.kRedTurningEncoderReversed,
                constants.kRedDriveAbsoluteEncoderPort,
                constants.kRedDriveAbsoluteEncoderOffsetRad,
                constants.kRedDriveAbsoluteEncoderReversed,
                constants.redDriveInvert,
                constants.redTurnInvert);

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

    // public double getTurningVelocity() {
    //     return pigeon.getAngularVelocityXDevice().getValue();
    // }
    // public Pose2d getPose() {
    // return odometer.getPoseMeters();
    // }

    // public void resetOdometry(Pose2d pose) {
    // odometer.resetPosition(getRotation2d(), null, pose);
    // }

    @Override
    public void periodic() {
        // note odometry settings commented out bc of swervedrivestate and
        // swervedriveposition
        // odometer.update(getRotation2d(), driveStates);
        SmartDashboard.putNumber("Robot Heading", getHeading());
        // SmartDashboard.putString("Robot Location",
        // getPose().getTranslation().toString());
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

    public Rotation2d getRotation2D() {
        //gets pigeon value for rotation in degrees, converts to radians
        double numDegrees = pigeon.getYaw().getValue();
        double radians = numDegrees * (Math.PI/180);
        return new Rotation2d(radians);
    }

    public void teleopControlSwerve(double leftX, double leftY, double rightX) {
        // value originally Math.atan(leftY/leftX)/(Math.PI *2), got rid of dividing by
        // pi
        Rotation2d desRot = new Rotation2d(Math.atan2(leftY, leftX));
        double velocity = leftY;

        // this works!!! time to scale the vectors!!
        // check if lefty is not 1 or -1 (full magnitudes)
        // SlewRateLimiter xLimiter = new SlewRateLimiter(0.5);
        // SlewRateLimiter yLimiter = new SlewRateLimiter(0.5);
        if (leftY < 0.05 && leftY > -0.05) {
            if (leftX >= 0.05) {
                desRot = new Rotation2d(90);
                leftX = xLimiter.calculate(leftX) * constants.kTeleDriveMaxSpeedMetersPerSecond;
            } else if (leftX <= -0.05) {
                desRot = new Rotation2d(-90);
            }
            velocity = leftX;
        }
        // try mult vs division
        leftY = yLimiter.calculate(leftX) * constants.kTeleDriveMaxSpeedMetersPerSecond;
        velocity = leftY;

        SwerveModuleState desiredState = new SwerveModuleState(velocity, desRot);

        // leave this alone this is the only thing that works
        driveStates[0] = desiredState;
        driveStates[1] = desiredState;
        driveStates[2] = desiredState;
        driveStates[3] = desiredState;

        blue.setDesiredState(desiredState);
        orange.setDesiredState(desiredState);
        green.setDesiredState(desiredState);
        red.setDesiredState(desiredState);

    }

    public void execute(double leftX, double leftY, double rightX) {
        // 1. Get real-time joystick inputs
        double xSpeed = leftX*5;
        double ySpeed = leftY*5;
        double turningSpeed = rightX;

        // 2. Apply deadband
        xSpeed = Math.abs(xSpeed) > constants.OIConstants ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > constants.OIConstants ? ySpeed : 0.0;
        turningSpeed = Math.abs(turningSpeed) > constants.OIConstants ? turningSpeed : 0.0;

        // 3. Make the driving smoother
        xSpeed = xLimiter.calculate(xSpeed) * constants.kTeleDriveMaxSpeedMetersPerSecond;
        ySpeed = yLimiter.calculate(ySpeed) * constants.kTeleDriveMaxSpeedMetersPerSecond;
        turningSpeed = turningLimiter.calculate(turningSpeed)
                * constants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

        // 4. Construct desired chassis speeds
        ChassisSpeeds chassisSpeeds;

        // ensures field orientation
        chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed, ySpeed, turningSpeed, getRotation2D());

        // 5. Convert chassis speeds to individual module states
        SwerveModuleState[] moduleStates = constants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        // 6. Output each module states to wheels
        this.setModuleStates(moduleStates);
    }

}