package frc.robot.commands.Autos;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.auto.RamseteAutoBuilder;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ArcadeDrive;

public class TestAuto extends SequentialCommandGroup{
    public TestAuto(RamseteAutoBuilder autoBuilder, ArcadeDrive drive){
        var pathTrajectory = PathPlanner.loadPath("TestPath", new PathConstraints(3.0, 2.0));

        var autoPath = autoBuilder.fullAuto(pathTrajectory);

    }
    
}
