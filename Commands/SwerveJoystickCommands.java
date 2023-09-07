package frc.robot.commands;

import java.util.function.Supplier;
import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveJoystickCommands extends CommandBase {

    private final SwerveSubsystem SwerveSubsystem;
    private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
    private final Supplier<Boolean> fieldOrientedFunction;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;

    public SwerveJoystickCommands(SwerveSubsystem swerveSubsystem, Supplier<Double> xSpdFunction, 
    Supplier<Double> ySpdFunction, Supplier>Double> turningSpdFunction, Supplier<Boolean> fieldOrientedFunction){
        this.swerveSubsystem = swerveSubsystem;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.turningSpdFunction = turningSpdFunction;
        this.fieldOrientedFunction = fieldOrientedFunction;

        //optional rate limiter stuff 
        this.xLimiter = new SlewRateLimiter(DriveConstatns.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.turningLimiter = new SlewRatelimiter(DivecConstants.kTeleDriveMaxAngularAcceleration;
        )
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        //1. get real time input from controller 
        double xSpeed = xSpdFunction.get();
        double ySpeed = ySpdFunction.get();
        
        double turningSpeed = turningSpd
        double turningSpeed = turningSpdFunction.get();

        //2. apply deadband (if controller doesn't center back to exactly 0, ignore small inputs to protect motors)
        xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0.0;
        turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ? turningSpeed : 0.0;

        //3. (OPTIONAL) add rate limiter to limit acceleration in case joystick is hit to violently
        //Basically stops from going 0 too 100 obnoxiously and makes driving smoother
        //labelled as optional in constructor - ask nick/carter for opinion eventually
        xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        turningSpeed = turningLimiter.calculate(turningSpeed)
                * DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond;

        //4. convert speeds to appropriate reference frames
        //if driver wants to operate in field reference, wpilib goes to local reference frame.
        //otherwise, make new speeds
        ChassisSpeeds chassisSpeeds;
        if (fieldOrientatedFunction.get()) {
            //Relative to field
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());
        }else {
            //Relative to robot
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
        }

        //5. Convert speeds to individual module states
        // Creates an array of 4 werve module states
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        //6. Set/output module states to each wheel
        swerveSubsystem.setModuleStates(moduleStates);
    }

    @Override
    public void end(boolean interrupted){
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;

    }
}