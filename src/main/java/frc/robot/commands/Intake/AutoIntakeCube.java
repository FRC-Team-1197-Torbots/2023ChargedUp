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

    public AutoIntakeCube(Intake subsystem,double percentoutput){
        this.intakeSystem = subsystem;
        cubespeed = percentoutput;

        addRequirements(subsystem);

    }

    @Override
    public void initialize(){
        intakeSystem.SetSolenoid(true);
    }
    @Override
    public void execute(){
        intakeSystem.SetRollerSpeed(cubespeed);
    }
}