package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Drive.LinearTrajectory;
import frc.robot.subsystems.DriveTrain;

public class OtherDumbAuto extends SequentialCommandGroup{
    public OtherDumbAuto(DriveTrain driveSubsystem){
        addCommands(new LinearTrajectory(driveSubsystem, 0.1, 0.35), new LinearTrajectory(driveSubsystem, -0.2, 4.5));
    }
    
}
