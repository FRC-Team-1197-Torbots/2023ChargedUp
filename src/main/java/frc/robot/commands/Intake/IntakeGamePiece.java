package frc.robot.commands.Intake;

import frc.robot.RobotContainer;
import frc.robot.Constants.ElevatorArmConstants.GamePiece;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
public class IntakeGamePiece extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Intake intakeSystem;
   
    //private double cubespeed;
    private double conespeed;
    private double cubespeed;
    private boolean state;
    //private GamePiece m_gamePiece;
    public IntakeGamePiece(Intake subsystem){//, GamePiece gamePiece){
        this.intakeSystem = subsystem;
        state = true;
        //m_gamePiece = gamePiece;
        addRequirements(subsystem);

    }

    @Override
    public void initialize(){
        intakeSystem.SetSolenoid(state);
    }
    @Override
    public void execute(){
        //intakeSystem.SetSolenoid(false);
        if(RobotContainer.player1_HoldButton.getRightBumperPressed()){
            state = !state;
            intakeSystem.SetSolenoid(state);
        if(RobotContainer.player1_HoldButton.getLeftBumper()){
            System.out.println("intake running");
            intakeSystem.SetRollerSpeed(0.2);
        }

        }
        //else{
        //intakeSystem.SetRollerSpeed(cubespeed);
        
    }
    

    @Override
    public void end(boolean interrupted){

    }

    @Override
    public boolean isFinished(){
        return false;
    }
}