package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Drive.LinearTrajectory;
import frc.robot.subsystems.DriveTrain;

public class DoNothing extends SequentialCommandGroup{
    public DoNothing(DriveTrain driveSubsystem){
        addCommands(new LinearTrajectory(driveSubsystem, 0, 5));
    }
    
}
