package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class LinearTrajectory extends CommandBase{
    
    private double timeInit;
    private double currentTime;
    private double elapsedTime;
    private double targetTime;
    private DriveTrain driveSubsystem;
    private double driveSpeed;

    public LinearTrajectory(DriveTrain driveSub, double speed, double runTime){
        elapsedTime = 0;
        driveSubsystem = driveSub;
        targetTime = runTime;
    }

    @Override
    public void initialize(){
        timeInit = Timer.getFPGATimestamp();
    }

    @Override
    public void execute(){
        currentTime = Timer.getFPGATimestamp();
        elapsedTime = currentTime - timeInit;
        driveSubsystem.setMotorSpeeds(driveSpeed, driveSpeed);
    }
    @Override
    public void end(boolean interrupted){

    }
    @Override
    public boolean isFinished(){
        return elapsedTime >= targetTime;
    }
}
    

