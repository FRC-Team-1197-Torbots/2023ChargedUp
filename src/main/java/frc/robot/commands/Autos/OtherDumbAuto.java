package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Drive.LinearTrajectory;
import frc.robot.subsystems.DriveTrain;

public class OtherDumbAuto extends SequentialCommandGroup{
    public OtherDumbAuto(DriveTrain driveSubsystem){
        //System.out.println("auto working");
        addCommands(new LinearTrajectory(driveSubsystem, -0.4, 3000), new PrintCommand("Auto DONE"));//.andThen(() -> driveSubsystem.Drive(0, 0)));
    }
    
}
