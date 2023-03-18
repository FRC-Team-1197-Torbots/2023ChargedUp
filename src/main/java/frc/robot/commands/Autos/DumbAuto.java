package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Drive.LinearTrajectory;
import frc.robot.subsystems.DriveTrain;

public class DumbAuto extends SequentialCommandGroup{


    //Temporary Auto for the LA Regional
    public DumbAuto(DriveTrain driveTrain){
        
        addCommands(new LinearTrajectory(driveTrain, -0.2, 4));
    }
    
}
