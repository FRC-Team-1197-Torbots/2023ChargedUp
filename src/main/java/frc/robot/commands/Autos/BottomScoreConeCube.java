package frc.robot.commands.Autos;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.RamseteAutoBuilder;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Utils.TrajectoryUtils;
import frc.robot.commands.Drive.ArcadeDrive;
import frc.robot.commands.Drive.SetMotorMode;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.commands.Autos.SetAuto;

public class BottomScoreConeCube extends SequentialCommandGroup{
    public BottomScoreConeCube(
        String pathName, 
        RamseteAutoBuilder autoBuilder,
        DriveTrain driveTrain,
        Intake intake,
        Elevator elevator,
        Arm arm){
            var pathTrajectory = PathPlanner.loadPath(pathName, 
            new PathConstraints(3.0, 2.5));
            //var pathTrajectory = TrajectoryUtils.readTrajectory(pathName, 
            

            var autoPath = autoBuilder.fullAuto(pathTrajectory);
            addCommands(new SetAuto(pathName, pathTrajectory), 
            autoPath,
            new SetMotorMode(driveTrain, IdleMode.kBrake).andThen(() -> driveTrain.AutoDriveMotor(0, 0))
            );
        }
    
}
// score "cone"
// drive to "other_cone"
// intake "other_cone"
// score "score_cone"
// #win
// #NikitaIsGOATed