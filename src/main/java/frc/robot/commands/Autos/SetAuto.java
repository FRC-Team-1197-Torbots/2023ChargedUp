package frc.robot.commands.Autos;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SetAuto extends CommandBase{
    private String m_pathName;
    PathPlannerTrajectory m_Trajectory;
    List<PathPlannerTrajectory> m_trajectoryList;
    
    public SetAuto(String pathName, PathPlannerTrajectory trajectory){
        m_pathName = pathName;
        m_Trajectory = trajectory;


    }

    @Override
    public void initialize(){
        var redAlliance = false;

        if(m_trajectoryList != null){
            ArrayList<PathPlannerTrajectory> m_pathTrajectories = new ArrayList<>();
            for(var traj: m_trajectoryList){
                m_pathTrajectories.add(PathPlannerTrajectory.transformTrajectoryForAlliance(traj,
                 DriverStation.Alliance.Red));
            }
        }
        else{
            if(redAlliance){
                m_Trajectory = PathPlannerTrajectory.transformTrajectoryForAlliance(m_Trajectory, DriverStation.Alliance.Red);
            }
        }
    }

    @Override
    public void execute(){

    }

    @Override
    public void end(boolean interrupted){

    }

    @Override
    public boolean isFinished(){
        return true;
    }
    
}
