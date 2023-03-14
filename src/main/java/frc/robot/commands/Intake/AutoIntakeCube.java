package frc.robot.commands.Intake;

import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
public class AutoIntakeCube extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Intake intakeSystem;
   
    //private double cubespeed;
    private double cubespeed;
    private boolean value;
    public AutoIntakeCube(Intake subsystem,double percentoutput, boolean solenoid){
        this.intakeSystem = subsystem;
        cubespeed = percentoutput;
        value = solenoid;
        addRequirements(subsystem);

    }

    @Override
    public void initialize(){

    }
    @Override
    public void execute(){
        intakeSystem.SetSolenoid(value);
        intakeSystem.SetRollerSpeed(cubespeed);
    }
    @Override
    public void end(boolean interrupted) {
            
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return false;
  
    }
}