//Swerve subsystem
package frc.robot;

import edu.wpi.first.wpilipj2.command.Command;
//import edu.wpi.first.wpilibj.XboxController;
import java.util.List;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {
    
    private final SwerveSubstem swerveSubsystem = new SwerveSubsystem();

    //confirm if still valid w xbox controller
    private final Joystick driverController = new Joystick(OIConstants.kDriverControllerPort); 
    
    //need to fill with methods for xbox controller not joystick
    public RobotContainer(){
        swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
            swerveSubsystem,
            //3 joystick axes
            () -> -driverJoytick.getRawAxis(OIConstants.kDriverYAxis),,
            () -> driverJoystick.getRawAxis(OIConstants.jDriverXAxis),
            () -> driverJoystick.getRawAxis(OIConstants.kDriverRotAxis),
            //oppositevalue of the button so by default it operates in the field's reference frame
            () -> !driverJoytick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx))); 
             
        configureButtonBindings();
    }

    //resets direction of field's reference frame 
    public void configureButtonBindings() {
        //short way to make a comand!!
        new JoystickButton(driverJoystick, 2).whenPressed(() -> swerveSubsystem.zeroHeading());
    }

    public Command getAutonomousCommand() {
        return null;
    }

    //getautonomouscommand would go here when we get to this point
}