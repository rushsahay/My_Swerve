package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

public class Drive extends Command {
    private Supplier<Double> xSpeedFunc, ySpeedFunc, turnSpeedFunc;
    private Supplier<Boolean> fieldOrientedFunc;
    private SlewRateLimiter xLimitor, yLimitor, turnLimitor;
    private SwerveSubsystem swerve;
    public Drive(Supplier<Double> xSpeed,Supplier<Double> ySpeed, Supplier<Double> turnSpeed,
    Supplier<Boolean> fieldOriented,SwerveSubsystem swerve){
        this.xSpeedFunc = xSpeed;
        this.ySpeedFunc = ySpeed;
        this.turnSpeedFunc = turnSpeed;
        this.fieldOrientedFunc = fieldOriented;
        this.swerve = swerve;
        xLimitor = new SlewRateLimiter(Constants.SwerveConstants.TELE_DRIVE_MAX_ACCELERATION);
        yLimitor = new SlewRateLimiter(Constants.SwerveConstants.TELE_DRIVE_MAX_ACCELERATION);
        turnLimitor = new SlewRateLimiter(Constants.SwerveConstants.TELE_DRIVE_MAX_ANGULAR_ACCELERATION);
        addRequirements(swerve);
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        double xSpeed = xSpeedFunc.get();
        double ySpeed = ySpeedFunc.get();
        double turnSpeed = turnSpeedFunc.get();

        xSpeed = Math.abs(xSpeed) > Constants.DeadBandConstants.xDeadband ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > Constants.DeadBandConstants.yDeadband ? ySpeed : 0.0;
        turnSpeed = Math.abs(turnSpeed) > Constants.DeadBandConstants.turnDeadband ? turnSpeed : 0.0;

        xSpeed = xLimitor.calculate(xSpeed)*Constants.SwerveConstants.TELE_DRIVE_MAX_SPEED;
        ySpeed = yLimitor.calculate(ySpeed)*Constants.SwerveConstants.TELE_DRIVE_MAX_SPEED;
        turnSpeed = turnLimitor.calculate(turnSpeed)*Constants.SwerveConstants.TELE_DRIVE_MAX_ANGULAR_SPEED;

        ChassisSpeeds chassisSpeeds;
        if(fieldOrientedFunc.get()){
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed,ySpeed,turnSpeed,swerve.getRotation2d());
                }
        else{
            chassisSpeeds = new ChassisSpeeds(xSpeed,ySpeed,turnSpeed);
        }

        SwerveModuleState[] desiredStates = Constants.SwerveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);

        swerve.setModuleStates(desiredStates);
    }
    
    @Override
    public void end(boolean interrupted){
        swerve.stopModules();
    }

    public boolean isFinished(){
        return false;
    }

}
