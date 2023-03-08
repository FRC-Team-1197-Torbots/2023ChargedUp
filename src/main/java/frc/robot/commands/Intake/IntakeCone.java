package frc.robot.commands.Intake;

import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
public class IntakeCone extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Intake intakeSystem;
   
    //private double cubespeed;
    private double conespeed;

    public IntakeCone(Intake subsystem){
        this.intakeSystem = subsystem;
        
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
}