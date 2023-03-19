package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Drive.LinearTrajectory;
import frc.robot.subsystems.DriveTrain;

public class DumbAuto extends CommandBase{
    private LinearTrajectory m_LinearTrajectory;

    //Temporary Auto for the LA Regional
    public DumbAuto(DriveTrain driveTrain){
        m_LinearTrajectory = new LinearTrajectory(driveTrain, 0.2, 4);
        m_LinearTrajectory.execute();
    }
    
}
