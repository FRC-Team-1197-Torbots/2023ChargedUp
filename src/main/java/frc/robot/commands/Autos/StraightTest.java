package frc.robot.commands.Autos;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.auto.RamseteAutoBuilder;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Drive.SetMotorMode;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;

public class StraightTest extends SequentialCommandGroup{
    public StraightTest(
        String pathName, 
        RamseteAutoBuilder autoBuilder,
        DriveTrain driveTrain,
        Intake intake,
        Elevator elevator,
        Arm arm){
            var pathTrajectory = PathPlanner.loadPath(pathName, 
            new PathConstraints(3.0, 2.5));
            //var pathTrajectory = TrajectoryUtils.readTrajectory(pathName, 
            System.out.println("Left Encoder Value: " + driveTrain.getLeftEncoder());
            System.out.println("Right Encoder Value: " + driveTrain.getRightEncoder());
            

            var autoPath = autoBuilder.fullAuto(pathTrajectory);
            addCommands(new SetAuto(pathName, pathTrajectory), autoPath,  
            new SetMotorMode(driveTrain, IdleMode.kBrake).andThen(() -> driveTrain.AutoDriveMotor(0, 0))
            );
        }
    
}
