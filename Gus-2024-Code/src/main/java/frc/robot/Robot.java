// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you 9 modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;

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

  CANSparkMax orangeDriveMotor = new CANSparkMax(36, MotorType.kBrushless);
  CANSparkMax orangeTurningMotor = new CANSparkMax(7, MotorType.kBrushless);

  CANSparkMax redDriveMotor = new CANSparkMax(4, MotorType.kBrushless);
  CANSparkMax redTurningMotor = new CANSparkMax(9, MotorType.kBrushless);

  CANSparkMax blueDriveMotor = new CANSparkMax(15, MotorType.kBrushless);
  CANSparkMax blueTurningMotor = new CANSparkMax(10, MotorType.kBrushless);

  CANSparkMax greenDriveMotor = new CANSparkMax(13, MotorType.kBrushless);
  CANSparkMax greenTurningMotor = new CANSparkMax(14, MotorType.kBrushless);

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

  /**
   * This function is called once each time the robot enters teleoperated mode.
   */
  @Override
  public void teleopInit() {
  }

  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {
    // m_robotDrive.arcadeDrive(m_stick.getY(), m_stick.getX());
    orangeDriveMotor.set(baseController.getLeftY() / 2);
    greenDriveMotor.set(baseController.getLeftY() / 2);
    redDriveMotor.set(baseController.getLeftY() / 2);
    blueDriveMotor.set(baseController.getLeftY() / 2);
  }

  /** This function is called once each time the robot enters test mode. */
  @Override
  public void testInit() {
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }
}
