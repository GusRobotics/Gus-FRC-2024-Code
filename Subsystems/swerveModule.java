//one motor controls speed
//one motor controls angle/direction of wheel

//three joystick inputs
        //xSpd
        //ySpd
        //angle

//speed is given in m/s and is converted to motor output percentage
        //divide incoming speed by max robot speed
        //for more accuracy you can use feedforward controls or a velocity pid (advanced)
//for angle we can use a pid controller
        //for optimizatiton use the "optimize motion function" to keep wheel from every spinning more than 90 degrees
        // (for example, spins -45 rather than 135)
=

package frc.robot.subsystem;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {
        private final CANSparkMax driveMotor;
        private final CANSparkMax turningMotor;

        //Access encoders built into the motors
        private final CANEncoder driveEncoder;
        private final CANEncoder turningEncoder;

        //Moves the angle motor (or we can use PID controller built-n to moro controller)
        private final PIDController turningPidController;

        //every time robot is powered off, the motor's enocders lose previous readings
        //Absolute Encoder allows robot to always figure out where robot's wheels are facing
        //connected to analog ports on roborio
        private final AnaLogInput absoluteEncoder;
        private final boolean absoluteEncoderReversed;
        //can store how off absolute encoder reading is from actual wheel angle
        private final double absoluteEncoderOffsetRad;

        public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, 
        boolean turningMotorReversed, int absoluteEncoderId, double abosluteEncoderOffset, boolean absoluteEncoderReversed)
        {
                //Creates absolute encoder
                this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
                this.absoluteEncoderReversed = absoluteEncoderReversed;
                absolute Encoder = new AnalogInput(absoluteEncoderId);

                //Creates motors
                driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
                turningMotor = new CANSparkMax(turningMotorId, MotorType.kBrushless);

                driveMotor.setInverted(driveMotorReversed);
                turningMotor.setInverted(turningMotorReversed);

                //Gets motor encoders
                driveEncoder = driveMotor.getEncoder();
                turningEncoder = turninigMotor.getEncoder();

                //Sets encoder conversion constants so we can work with meters and radians instead of rotations
                driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
                driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
                turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
                turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRRPM2RadPerSec);

                turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);
                turningPidController.enableContinuousInput(-Math.PI, Math.PI);

                resetEncoders();

        }
}
//helpful methods to get encoder values
//About Built in Encoders
        public double getDrivePosition() {
                return driveEncoder.getPosition();
        }

//About Built in Encoders
        public double getTurningPosition() {
                return turningEncoder.getPosition();
        }

//About Built in Encoders
        public double getDriveVelocity() {
                return driveEncoder.getVelocity();
        }

//About Built in Encoders
        public double getTurningVelocity() {
                return turningEncoder.getVelocity();
        }

//For this you need o divide voltage reading by voltage we're supplying
//Gives percentage of a full rotation it's reading
        public double getAbsoluteEncoderRad() {
                double angle = absoluteEncoder.getVoltage() /RobotController.getVoltage5V();
                //multiplying by 2pi to convert to rad
                angle *= 2.0 * Math.PI;
                //subtract offset to get actual wheel angles
                angle -= absoluteEncoderRad;
                //multiply by -1 if reversed //FIX/CONFIRM SYNTAX BELOW
                return angle * (absoluteEncoderReversed ? -1.0: 1.0)
        }

//Function to give motors the values of the absolute encoder which always know their locations bc they always reset
//Reset drive motor encoder to 0 and the turning encoder to be the absolute encoder value
//Turning encoder's reading si aligned with wheel's actual angle
//We call this function when the robot boots up
        public void resetEncoders() {
                driveEncoder.setPosition(0);
                turningEncoder.setposition(getAbsoluteEncoderRad());
        }

//method returns info in the form of "swerve module state" how wpilib likes it
        public SwerveModuleState getState() {
                return new SwerveModuleState(getDriveVelocity(), new  Rotation2d(getTurningPosition))
        }

//function to actuate this module
        public void setDesiredState(SwerveModuleState state) { {
                //as soon as you let go of controller wpilib resets module state back to 0 degrees
                //if statement below prevents this
                //checks if there's no substantial driving velocity to determine if we can ignore the resetting
                if (Math.abs(state.speedMetersPerSecond) < 0.001) {
                        stop();
                        return;
                }

                //optimize angle setpoint so we never have to move more than 90 degrees
                state = SwervemoduleState.optimize(state, getState().angle);
                driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
                //calculate output for angle setpoint and turn position
                turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
                //sends out debug information
                SmartDashboard.putString("Swerve[" = absoluteEncoder.getChannel() = "] state", state.toString());

        }

        //stop function to keep wheels from resetting when robot is still
        public void stop() {
                driveMotor.set(0);
                turningMotor.set(0);
        }

}