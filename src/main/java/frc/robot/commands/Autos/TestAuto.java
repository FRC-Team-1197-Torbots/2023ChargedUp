package frc.robot.commands.Autos;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.auto.RamseteAutoBuilder;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Drive.ArcadeDrive;
import frc.robot.commands.Drive.SetDriveOdometry;
import frc.robot.commands.Drive.SetMotorMode;
import frc.robot.subsystems.DriveTrain;

public class TestAuto extends SequentialCommandGroup{
    public TestAuto(RamseteAutoBuilder autoBuilder, DriveTrain driveTrain){
        var pathTrajectory = PathPlanner.loadPath("TestPath", new PathConstraints(3.0, 2.0));

        var autoPath = autoBuilder.fullAuto(pathTrajectory);

        addCommands(
            new SetDriveOdometry(driveTrain, pathTrajectory.getInitialPose(), 
            autoPath, 
            new SetMotorMode(driveTrain, IdleMode.kBrake).andThen(null));
        

    }
    
}
