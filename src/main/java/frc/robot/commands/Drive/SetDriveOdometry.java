package frc.robot.commands.Drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class SetDriveOdometry extends CommandBase{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final DriveTrain m_DriveTrain;
    private Pose2d m_Pose2d;

    public SetDriveOdometry(DriveTrain drive_train, Pose2d pose2d){
        m_DriveTrain = drive_train;
        m_Pose2d = pose2d;
        addRequirements(drive_train);
    }

    @Override
    public void initialize(){
        m_DriveTrain.setOdometry(m_Pose2d);
        SmartDashboard.putNumber("DriveInitialPositionX", m_Pose2d.getX());
        SmartDashboard.putNumber("DriveInitialPositionY", m_Pose2d.getY());
        SmartDashboard.putNumber("DriveInitialPositionRotation", m_Pose2d.getRotation().getDegrees());
    }

    @Override
    public boolean isFinished() {
      return true;
    }
    
}
