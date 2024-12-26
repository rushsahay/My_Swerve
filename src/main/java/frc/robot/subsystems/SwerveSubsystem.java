package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveSubsystem extends SubsystemBase {
    public final SwerveModule leftFront = new SwerveModule(Constants.SwerveConstants.LEFT_FRONT_DRIVE_ID,
    Constants.SwerveConstants.LEFT_FRONT_TURN_ID,
    Constants.SwerveConstants.LEFT_FRONT_DRIVE_MOTOR_REVERSED,
    Constants.SwerveConstants.LEFT_FRONT_TURN_MOTOR_REVERSED,
    Constants.SwerveConstants.LEFT_FRONT_CANCODER_ID,
    Constants.SwerveConstants.LEFT_FRONT_CANCODER_REVERSED,
    Constants.SwerveConstants.LEFT_FRONT_OFFSET);

    public final SwerveModule rightFront = new SwerveModule(Constants.SwerveConstants.RIGHT_FRONT_DRIVE_ID,
    Constants.SwerveConstants.RIGHT_FRONT_TURN_ID,
    Constants.SwerveConstants.RIGHT_FRONT_DRIVE_MOTOR_REVERSED,
    Constants.SwerveConstants.RIGHT_FRONT_TURN_MOTOR_REVERSED,
    Constants.SwerveConstants.RIGHT_FRONT_CANCODER_ID,
    Constants.SwerveConstants.RIGHT_FRONT_CANCODER_REVERSED,
    Constants.SwerveConstants.RIGHT_FRONT_OFFSET);

    public final SwerveModule leftBack = new SwerveModule(Constants.SwerveConstants.LEFT_BACK_DRIVE_ID,
    Constants.SwerveConstants.LEFT_BACK_TURN_ID,
    Constants.SwerveConstants.LEFT_BACK_DRIVE_MOTOR_REVERSED,
    Constants.SwerveConstants.LEFT_BACK_TURN_MOTOR_REVERSED,
    Constants.SwerveConstants.LEFT_BACK_CANCODER_ID,
    Constants.SwerveConstants.LEFT_BACK_CANCODER_REVERSED,
    Constants.SwerveConstants.LEFT_BACK_OFFSET);

    public final SwerveModule rightBack = new SwerveModule(Constants.SwerveConstants.RIGHT_BACK_DRIVE_ID,
    Constants.SwerveConstants.RIGHT_BACK_TURN_ID,
    Constants.SwerveConstants.RIGHT_BACK_DRIVE_MOTOR_REVERSED,
    Constants.SwerveConstants.RIGHT_BACK_TURN_MOTOR_REVERSED,
    Constants.SwerveConstants.RIGHT_BACK_CANCODER_ID,
    Constants.SwerveConstants.RIGHT_BACK_CANCODER_REVERSED,
    Constants.SwerveConstants.RIGHT_BACK_OFFSET);
    private final StructArrayPublisher<SwerveModuleState> actualSwerve;
    private final Pigeon2 gyro = new Pigeon2(Constants.SwerveConstants.PIGEON_ID);
    private final Field2d field;
    private final SwerveDriveOdometry odom = new SwerveDriveOdometry(Constants.SwerveConstants.DRIVE_KINEMATICS, new Rotation2d(0),getModulePositions());
    private final SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(Constants.SwerveConstants.DRIVE_KINEMATICS,
     getRotation2d(), 
     getModulePositions(), 
     getPose2d());
    public SwerveSubsystem(){
        new Thread(()->{
        try{
            Thread.sleep(1000);
            resetHeading();
        }
        catch(Exception e){

        }}).start();

        actualSwerve = NetworkTableInstance.getDefault().getTable("24K").getStructArrayTopic("Actual_States", SwerveModuleState.struct).publish();
        field = new Field2d();
        SmartDashboard.putData("GWR Field",field);
    }
    public void resetHeading(){
        gyro.reset();
        resetPose();
    }
    public void zeroHeading(){
        gyro.setYaw(0);
        resetPose();
    }

    public void resetEncoders(){
        leftFront.resetEncoders();
        rightFront.resetEncoders();
        leftBack.resetEncoders();
        rightBack.resetEncoders();
        resetPose();
    }
    public double getHeading(){
        return Math.IEEEremainder(gyro.getAngle(), 360);
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] modulePositions = {new SwerveModulePosition(leftFront.getDriverPosition(),leftFront.getRotation2d()),
            new SwerveModulePosition(rightFront.getDriverPosition(),rightFront.getRotation2d()),
        new SwerveModulePosition(leftBack.getDriverPosition(),leftBack.getRotation2d()),
    new SwerveModulePosition(rightBack.getDriverPosition(),rightBack.getRotation2d())};
    return modulePositions;
    }

    public Rotation2d getRotation2d(){
        return new Rotation2d().fromDegrees(getHeading());
    }
    public Pose2d getPose2d(){
        return odom.getPoseMeters();
    }

    public void resetPose(){
        odom.resetPosition(getRotation2d(), getModulePositions(), poseEstimator.getEstimatedPosition());
    }

    public void setPose(Pose2d pose){
        odom.resetPosition(getRotation2d(), getModulePositions(),pose);
    }
    public SwerveModuleState[] getSwerveStates(){
        SwerveModuleState[] states = {leftFront.getState(),rightFront.getState(),leftBack.getState(),rightBack.getState()};
        return states;
    }
    @Override
    public void periodic(){
        odom.update(getRotation2d(),getModulePositions());
        actualSwerve.set(getSwerveStates());
        poseEstimator.update(getRotation2d(),getModulePositions());
        SmartDashboard.putNumber("Robot Heading",getHeading());
        field.setRobotPose(poseEstimator.getEstimatedPosition());
    }

    public void stopModules(){
        leftFront.stop();
        rightFront.stop();
        leftBack.stop();
        rightBack.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates){
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates,Constants.SwerveConstants.TELE_DRIVE_MAX_SPEED);
        leftFront.setSwerveState(desiredStates[0]);
        rightFront.setSwerveState(desiredStates[1]);
        leftBack.setSwerveState(desiredStates[2]);
        rightBack.setSwerveState(desiredStates[3]);
    }
}
