package frc.robot.commands;

import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
public class MoveIntake extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Intake intakeSystem;
   
    private double cubespeed;
    private double conespeed;

    public MoveIntake(Intake subsystem){
        this.intakeSystem = subsystem;
        
        addRequirements(subsystem);

    }
    @Override
    public void execute(){
        if (RobotContainer.player1.getAButtonPressed()){
            intakeSystem.SetSolenoid(true);
        }
        else{
            intakeSystem.SetSolenoid(false);

        }
        if (RobotContainer.player1.getLeftBumperPressed()){
            intakeSystem.SetRollerSpeed(cubespeed);
        }
        else if (RobotContainer.player1.getRightBumperPressed()){
            intakeSystem.SetRollerSpeed(conespeed);
        }
        else {
            intakeSystem.SetRollerSpeed(0);
        }
    }
}