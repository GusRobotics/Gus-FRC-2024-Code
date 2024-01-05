package frc.robot;

//import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
//import com.revrobotics.CANEncoder;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
//import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
// import edu.wpi.first.wpilibj.AnalogInput;
// import edu.wpi.first.wpilibj.RobotController;
//import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import com.ctre.phoenix.sensors.WPI_Pigeon2;
//import com.ctre.phoenix.sensors.WPI_CANCoder;
//import edu.wpi.first.wpilibj.XboxController;
import com.ctre.phoenix6.hardware.CANcoder;

public class SwerveModule {

    private final CANSparkMax driveMotor;
    private final CANSparkMax turningMotor;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turningEncoder;

    private final PIDController turningPidController;

    private final CANcoder absoluteEncoder;
    //private final boolean absoluteEncoderReversed;
    //private final double absoluteEncoderOffsetRad;


    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
            int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed, boolean reversedDrive, boolean reversedTurn) {

        // this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        // this.absoluteEncoderReversed = absoluteEncoderReversed;
        //swtiched AbsoluteEncoder to type CANcoder --> broke the getVoltage() and getChannel() methods
        absoluteEncoder = new CANcoder(absoluteEncoderId);
        //absoluteEncoder = new AnalogInput(absoluteEncoderId);

        driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
        turningMotor = new CANSparkMax(turningMotorId, MotorType.kBrushless);


        driveMotor.setInverted(driveMotorReversed);
        turningMotor.setInverted(turningMotorReversed);

        driveEncoder = driveMotor.getEncoder();
        turningEncoder = turningMotor.getEncoder();

        // driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
        // driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
        // turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
        // turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);

        turningPidController = new PIDController(constants.kPTurning, 0, 0);
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);

        driveMotor.setSmartCurrentLimit(constants.driveMotorCurrentLimit);
        turningMotor.setSmartCurrentLimit(constants.driveMotorCurrentLimit);
            
        driveMotor.setInverted(reversedDrive);
        turningMotor.setInverted(reversedTurn);

        resetEncoders();
    }

    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }

    // public CANSparkMax getDriveMotor(int driveId) {
    //     return new CANSparkMax(driveId, MotorType.kBrushless);
    // }

    // public CANSparkMax getSteeringMotor(int steerId) {
    //     return new CANSparkMax(steerId, MotorType.kBrushless);
    // }

    public double getTurningPosition() {
        return (turningEncoder.getPosition()*(1.0/(150.0/7.0)))*Math.PI*2;
    }

    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    public double getTurningVelocity() {
        return turningEncoder.getVelocity();
    }

    public double getAbsoluteEncoderRad() {
        // double angle = absoluteEncoder.getVoltage() / RobotController.getVoltage5V();
        // angle *= 2.0 * Math.PI;
        // angle -= absoluteEncoderOffsetRad;
        // //return angle;
        // return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
        return absoluteEncoder.getAbsolutePosition().getValue();
    }

    public void resetEncoders() {
        driveEncoder.setPosition(0);
        turningEncoder.setPosition(0);
    }

    //switched from return type swervemoduleposition to swervemodulestate for functionality
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(state.speedMetersPerSecond / constants.kPhysicalMaxSpeedMetersPerSecond);
        turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
        //SmartDashboard.putString("Swerve[" + absoluteEncoder.getChannel() + "] state", state.toString());
    }

    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }

}