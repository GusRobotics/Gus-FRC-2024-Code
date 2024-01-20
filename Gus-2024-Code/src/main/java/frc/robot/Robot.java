package frc.robot;

import com.ctre.phoenix6.hardware.core.CorePigeon2;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//FOR THE FIRST MEETING AFTER KICKOFF:: 
//write method to read the cancoder values in SwerveDrive, use in robotinit to display to smartdashboard, 
//callibrate values for cancodes for each of the modules somehow (idk average?) find reset values in test, 
//set that value as an automatic reset in robotinit

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

  SwerveDrive driveBase = new SwerveDrive();
  SwerveModuleState driveStates[] = new SwerveModuleState[4];

  CANcoder blueCan = new CANcoder(3);
  CANcoder orangeCan = new CANcoder(1);
  CANcoder greenCan = new CANcoder(5);
  CANcoder redCan = new CANcoder(2);

  private final CorePigeon2 pigeon = new CorePigeon2(constants.kPigeonPort);

  SwerveModule blue = new SwerveModule(
    constants.blueDrive,
    constants.blueSteer,
    constants.kBlueDriveEncoderReversed,
    constants.kBlueTurningEncoderReversed,
    constants.kBlueDriveAbsoluteEncoderPort,
    constants.kBlueDriveAbsoluteEncoderOffsetRad,
    constants.kBlueDriveAbsoluteEncoderReversed,
    constants.blueDriveInvert,
    constants.blueTurnInvert);

    SwerveModule orange = new SwerveModule(
    constants.orangeDrive,
    constants.orangeSteer,
    constants.kOrangeDriveEncoderReversed,
    constants.kOrangeTurningEncoderReversed,
    constants.kOrangeDriveAbsoluteEncoderPort,
    constants.kOrangeDriveAbsoluteEncoderOffsetRad,
    constants.kOrangeDriveAbsoluteEncoderReversed,
    constants.orangeDriveInvert,
    constants.orangeTurnInvert);

    SwerveModule green = new SwerveModule(
    constants.greenDrive,
    constants.greenSteer,
    constants.kGreenTurningEncoderReversed,
    constants.kGreenTurningEncoderReversed,
    constants.kGreenDriveAbsoluteEncoderPort,
    constants.kGreenDriveAbsoluteEncoderOffsetRad,
    constants.kGreenDriveAbsoluteEncoderReversed,
    constants.greenDriveInvert,
    constants.greenTurnInvert);

    SwerveModule red = new SwerveModule(
    constants.redDrive,
    constants.redSteer,
    constants.kRedDriveEncoderReversed,
    constants.kRedTurningEncoderReversed,
    constants.kRedDriveAbsoluteEncoderPort,
    constants.kRedDriveAbsoluteEncoderOffsetRad,
    constants.kRedDriveAbsoluteEncoderReversed,
    constants.redDriveInvert,
    constants.redTurnInvert);

  //RelativeEncoder blue = new RelativeEncoder(3);

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {

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
    // apply deadband once the code is written
    // if(Math.pow(baseController.getRightX(), 2) +
    // Math.pow(baseController.getRightX(), 2) >= 0.5){
    // if(baseController.getRightX() > 0){
    // desRot = new
    // Rotation2d(Math.atan(baseController.getRightY()/baseController.getRightX()));
    // }
    // else{
    // desRot = new Rotation2d(Math.PI -
    // Math.atan(baseController.getRightY()/baseController.getRightX()));
    // }
    // }
    // driveBase.teleopControlSwerve(baseController.getLeftX(),
    // baseController.getLeftY(), baseController.getRightX());
    driveBase.execute(baseController.getLeftX(), baseController.getLeftY(), baseController.getRightX());
    SmartDashboard.putNumber("sanity", 5);
    SmartDashboard.putNumber("Desired Rotation",
        Math.atan(baseController.getRightY() / baseController.getRightX()) / (Math.PI * 2));
    driveBase.periodic();

  }

  /** This function is called once each time the robot enters test mode. */
  @Override
  public void testInit() {
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    // driveBase.periodic();
    // driveBase.execute(baseController.getLeftX(), baseController.getLeftY(), baseController.getRightX());
    SmartDashboard.putNumber("sanity", 5);
    SmartDashboard.putNumber("blue cancoder", blueCan.getAbsolutePosition().getValue());
    SmartDashboard.putNumber("red cancoder", redCan.getAbsolutePosition().getValue());
    SmartDashboard.putNumber("orange cancoder", orangeCan.getAbsolutePosition().getValue());
    SmartDashboard.putNumber("green cancoder", greenCan.getAbsolutePosition().getValue());
    SmartDashboard.putNumber("pigeon yaw value", pigeon.getYaw().getValue());

  }
}