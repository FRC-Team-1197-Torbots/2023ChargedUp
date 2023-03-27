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
    private double m_distance;

    public LinearTrajectory(DriveTrain driveSub, double speed, double distance){
        //elapsedTime = 0;
        driveSpeed = speed;
        driveSubsystem = driveSub;
        m_distance = distance;
        //targetTime = runTime;
    }

    @Override
    public void initialize(){
        //timeInit = Timer.getFPGATimestamp();
    }

    @Override
    public void execute(){
        //currentTime = Timer.getFPGATimestamp();
        //elapsedTime = currentTime - timeInit;
        driveSubsystem.setMotorSpeeds(driveSpeed, driveSpeed);
        
    }
    @Override
    public void end(boolean interrupted){

    }
    @Override
    public boolean isFinished(){
        return (driveSubsystem.getAverageEncoder() > m_distance);
    }
}
    

