package frc.robot;

import java.util.List;

//import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
//import frc.robot.commands.SwerveJoystickCmd;
//import edu.wpi.first.wpilibj2.command.button.Button; DEPRECATED
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {
        private final SwerveDrive swerveSubsystem = new SwerveDrive();

        private Joystick driverJoytick = new Joystick(constants.kDriverControllerPort);

        // private Trigger whenPressed;

        public RobotContainer() {
                swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(swerveSubsystem,
                                () -> -driverJoytick.getRawAxis(constants.kDriverYAxis),
                                () -> driverJoytick.getRawAxis(constants.kDriverXAxis),
                                () -> driverJoytick.getRawAxis(constants.kDriverRotAxis),
                                () -> !driverJoytick.getRawButton(constants.kDriverFieldOrientedButtonIdx)));

                configureButtonBindings();
        }

        private void configureButtonBindings() {
                new JoystickButton(driverJoytick, 0).onTrue(new InstantCommand(() -> swerveSubsystem.zeroHeading()));
        }

        // private Trigger extracted() {
        // return whenHeld = new JoystickButton(driverJoytick, 2).whileTrue(() ->
        // swerveSubsystem.zeroHeading());
        // }

        public final class AutoConstants {
                public static final double kMaxSpeedMetersPerSecond = 3.0;
                public static final double kMaxAccelerationMetersPerSecondSquared = 2.0;
                public static final double kPXController = 0.5;
                public static final double kPYController = 0.5;
                public static final double kPThetaController = 0.5;
                // idk what the controller constraints is calling??? looked for it in cosntants
                // and its not there
                // public static final TrapezoidProfile kThetaControllerConstraints = new
                // TrapezoidProfile(2.0, 4.0);
                // Other constants...
        }

        public Command getAutonomousCommand() {
                // 1. Create trajectory settings
                TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
                                AutoConstants.kMaxSpeedMetersPerSecond,
                                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                                .setKinematics(constants.kDriveKinematics);

                // 2. Generate trajectory
                Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                                new Pose2d(0, 0, new Rotation2d(0)),
                                List.of(
                                                new Translation2d(1, 0),
                                                new Translation2d(1, -1)),
                                new Pose2d(2, -1, Rotation2d.fromDegrees(180)),
                                trajectoryConfig);

                // 3. Define PID controllers for tracking trajectory
                // resolving errors to build and properly install ctre api
                // WPI_Pigeon2 pigeon = new WPI_Pigeon2(10);
                PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
                PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
                ProfiledPIDController thetaController = new ProfiledPIDController(
                                AutoConstants.kPThetaController, 0, 0, constants.kThetaControllerConstraints);
                thetaController.enableContinuousInput(-Math.PI, Math.PI);

                // 4. Construct command to follow trajectory
                SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
                                trajectory,
                                swerveSubsystem::getPose,
                                constants.kDriveKinematics,
                                xController,
                                yController,
                                thetaController,
                                swerveSubsystem::setModuleStates,
                                swerveSubsystem);

                // 5. Add some init and wrap-up, and return everything
                return new SequentialCommandGroup(
                                new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectory.getInitialPose())),
                                swerveControllerCommand,
                                new InstantCommand(() -> swerveSubsystem.stopModules()));
        }
}