package frc.robot.commands.Intake;

import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
public class AutoIntakeCone extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Intake intakeSystem;
   
    //private double cubespeed;
    private double conespeed;

    public AutoIntakeCone(Intake subsystem, double percentoutput){
        this.intakeSystem = subsystem;
        conespeed = percentoutput;

        addRequirements(subsystem);

    }

    @Override
    public void initialize(){
        intakeSystem.SetSolenoid(true);
    }
    @Override
    public void execute(){
        intakeSystem.SetRollerSpeed(conespeed);
    }
    @Override
    public void end(boolean interrupted) {
        intakeSystem.SetSolenoid(false);
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return false;
  
    }
}