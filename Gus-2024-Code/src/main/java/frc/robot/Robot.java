// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you 9 modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.math.kinematics.SwerveModulePosition;
/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the manifest
 * file in the resource
 * directory.
 */
public class Robot extends TimedRobot {
  XboxController baseController = new XboxController(0);
  Rotation2d desRot = new Rotation2d(0);

  // CANSparkMax orangeDriveMotor = new CANSparkMax(36, MotorType.kBrushless);
  // CANSparkMax orangeTurningMotor = new CANSparkMax(7, MotorType.kBrushless);

  // CANSparkMax redDriveMotor = new CANSparkMax(4, MotorType.kBrushless);
  // CANSparkMax redTurningMotor = new CANSparkMax(9, MotorType.kBrushless);

  // CANSparkMax blueDriveMotor = new CANSparkMax(15, MotorType.kBrushless);
  // CANSparkMax blueTurningMotor = new CANSparkMax(10, MotorType.kBrushless);

  // CANSparkMax greenDriveMotor = new CANSparkMax(13, MotorType.kBrushless);
  // CANSparkMax greenTurningMotor = new CANSparkMax(14, MotorType.kBrushless);

  // private final SwerveModule blue = new SwerveModule(
  //           constants.blueDrive,
  //           constants.blueSteer,
  //           constants.kBlueDriveEncoderReversed,
  //           constants.kBlueTurningEncoderReversed,
  //           constants.kBlueDriveAbsoluteEncoderPort,
  //           constants.kBlueDriveAbsoluteEncoderOffsetRad,
  //           constants.kBlueDriveAbsoluteEncoderReversed);

  //   private final SwerveModule orange = new SwerveModule(
  //           constants.orangeDrive,
  //           constants.orangeSteer,
  //           constants.kOrangeDriveEncoderReversed,
  //           constants.kOrangeTurningEncoderReversed,
  //           constants.kOrangeDriveAbsoluteEncoderPort,
  //           constants.kOrangeDriveAbsoluteEncoderOffsetRad,
  //           constants.kOrangeDriveAbsoluteEncoderReversed);

  //   private final SwerveModule green = new SwerveModule(
  //           constants.greenDrive,
  //           constants.greenSteer,
  //           constants.kGreenTurningEncoderReversed,
  //           constants.kGreenTurningEncoderReversed,
  //           constants.kGreenDriveAbsoluteEncoderPort,
  //           constants.kGreenDriveAbsoluteEncoderOffsetRad,
  //           constants.kGreenDriveAbsoluteEncoderReversed);

  //   private final SwerveModule red = new SwerveModule(
  //           constants.redDrive,
  //           constants.redSteer,
  //           constants.kRedDriveEncoderReversed,
  //           constants.kRedTurningEncoderReversed,
  //           constants.kRedDriveAbsoluteEncoderPort,
  //           constants.kRedDriveAbsoluteEncoderOffsetRad,
  //           constants.kRedDriveAbsoluteEncoderReversed);

  SwerveDrive driveBase = new SwerveDrive();
  SwerveModuleState driveStates[] = new SwerveModuleState[4];

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    // orange.setInverted(true);
    // red.setInverted(true);

  }

  /** This function is run once each time the robot enters autonomous mode. */
  @Override
  public void autonomousInit() {
    // m_timer.reset();
    // m_timer.start();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    // Drive for 2 seconds
    // if (m_timer.get() < 2.0) {
    // m_robotDrive.arcadeDrive(0.5, 0.0); // drive forwards half speed
    // } else {
    // m_robotDrive.stopMotor(); // stop robot
    // }
  }
//GREEN AND ORANGE NEED REVERSING
  /**
   * This function is called once each time the robot enters teleoperated mode.
   */
  @Override
  public void teleopInit() {

  }

  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {
    if(Math.pow(baseController.getRightX(), 2) + Math.pow(baseController.getRightX(), 2) >= 0.5){
      if(baseController.getRightX() > 0){
        desRot = new Rotation2d(Math.atan(baseController.getRightY()/baseController.getRightX()));
      }
      else{
        desRot = new Rotation2d(Math.PI - Math.atan(baseController.getRightY()/baseController.getRightX()));
      }
    }

    SwerveModuleState desiredState = new SwerveModuleState(baseController.getLeftY()*2, desRot);

    driveStates[0] = desiredState; 
    driveStates[1] = desiredState;
    driveStates[2] = desiredState;
    driveStates[3] = desiredState;
    driveBase.setModuleStates(driveStates);

    SmartDashboard.putNumber("sanity", 5);
    SmartDashboard.putNumber("Desired Rotation", Math.atan(baseController.getRightY()/baseController.getRightX())/(Math.PI *2));
    driveBase.periodic();
  }

  /** This function is called once each time the robot enters test mode. */
  @Override
  public void testInit() {
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    driveBase.periodic();
    SmartDashboard.putNumber("sanity", 5);
  }
}
