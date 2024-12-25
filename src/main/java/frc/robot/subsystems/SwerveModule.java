package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class SwerveModule{
    private CANSparkMax driveMotor;
    private CANSparkMax turnMotor;

    private CANEncoder driveEncoder;
    private CANEncoder turnEncoder;

    private final PIDController turningPID;

    private AnalogInput absoluteEncoder;
    private boolean absReverse;
    private double absOffset;

    public SwerveModule(int driveMotorID, int turnMotorID, boolean driveMotorReversed, boolean turnMotorReversed, int absEncoderID,
    boolean absReverse, double absOffset){
        this.absReverse = absReverse;
        this.absOffset = absOffset;
        absoluteEncoder = new AnalogInput(absEncoderID);

        driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
        turnMotor = new CANSparkMax(turnMotorID, MotorType.kBrushless);

        driveMotor.setInverted(driveMotorReversed);
        turnMotor.setInverted(turnMotorReversed);

        driveEncoder = driveMotor.getEncoder();
        turnEncoder = turnMotor.getEncoder();

        driveEncoder.setPositionConversionFactor(Constants.SwerveConstants.DRIVE_ENCODER_ROT2METERS);
        driveEncoder.setVelocityConversionFactor(Constants.SwerveConstants.DRIVE_ENCODER_ROT2METERPERSEC);

        turnEncoder.setPositionConversionFactor(Constants.SwerveConstants.TURNING_ENCODER_ROT2RAD);
        turnEncoder.setVelocityConversionFactor(Constants.SwerveConstants.TURNING_ENCODER_ROT2RADPERSEC);

        turningPID = new PIDController(Constants.SwerveConstants.KP_TURNING,0,0);
        turningPID.enableContinuousInput(-Math.PI, Math.PI);

        resetEncoders();
    }
    public double getDriverPosition(){
        return driveEncoder.getPosition();
    }
    public double getTurningPosition(){
        return turnEncoder.getPosition();
    }
    public double getDriverVelocity(){
        return driveEncoder.getVelocity();
    }
    public double getTurningVelocity(){
        return turnEncoder.getVelocity();
    }
    public Rotation2d getRotation2d(){
        return new Rotation2d(turnEncoder.getPosition());
    }
    public double getAbsoluteEncoderRad(){
        double angle = absoluteEncoder.getVoltage()/RobotController.getVoltage5V();
        angle*=2.0*Math.PI;
        angle -=absOffset;
        if(absReverse)
        return angle*-1.0;
        return angle;
    }

    public void resetEncoders(){
        driveEncoder.setPosition(0);
        turnEncoder.setPosition(getAbsoluteEncoderRad());
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(getDriverVelocity(), new Rotation2d(getTurningPosition()));
    }

    public void setSwerveState(SwerveModuleState desiredState){
        if(desiredState.speedMetersPerSecond<0.001){
            stop();
            return;
        }
        desiredState = SwerveModuleState.optimize(desiredState, getState().angle);
        driveMotor.set(desiredState.speedMetersPerSecond/Constants.SwerveConstants.TELE_DRIVE_MAX_SPEED);
        turnMotor.set(turningPID.calculate(getTurningPosition(),desiredState.angle.getRadians()));
        SmartDashboard.putString("Swerve["+absoluteEncoder.getChannel()+"] State",desiredState.toString());
    }
    public void stop(){
        driveMotor.set(0);
        turnMotor.set(0);
    }
}