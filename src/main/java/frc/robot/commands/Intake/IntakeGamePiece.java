package frc.robot.commands.Intake;

import frc.robot.RobotContainer;
import frc.robot.Constants.ElevatorArmConstants.GamePiece;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
public class IntakeGamePiece extends CommandBase {

    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Intake intakeSystem;
   
    //private double cubespeed;
    private double conespeed;
    private double cubespeed;
    private boolean state;
    
    public static enum HopperPiece{
        CONE, CUBE
    }
    private enum IntakeState{
        UP, DOWN
    }

    public static HopperPiece m_HopperState;
    private IntakeState m_IntakeState;
    //private GamePiece m_gamePiece;
    public IntakeGamePiece(Intake subsystem){//, GamePiece gamePiece){
        this.intakeSystem = subsystem;
        state = true;
        m_IntakeState = IntakeState.UP;
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
        //intakeSystem.SetRollerSpeed(0.3);
        
        if(RobotContainer.player1_HoldButton.getRightBumperPressed()){
            if(m_IntakeState == IntakeState.DOWN){
                m_IntakeState = IntakeState.UP;
            }
            else{
                m_IntakeState = IntakeState.DOWN;
            }
        }
        if(RobotContainer.player2_HoldButton.getLeftTriggerAxis() >0.5){
            //System.out.println("bumper pressed");
            m_HopperState = HopperPiece.CONE;
        }
        switch(m_IntakeState){
            case UP:
                //System.out.println("Intake up");
                state = true;
                intakeSystem.SetRollerSpeed(0, 0);
                Timer.delay(0.5);
                intakeSystem.SetSolenoid(state);
                //System.out.println("intake running");
                break;
            case DOWN:
                //System.out.println("Intake down");
                state = false;
                intakeSystem.SetSolenoid(state);
                Timer.delay(0.5);
                //System.out.println("intake off");
                intakeSystem.SetRollerSpeed(0.5, 0.6);//Cube speed: top = 0.5, bot = 0.6
                 
                break;
        }
            
            
        

    }
        //else{
        //intakeSystem.SetRollerSpeed(cubespeed);
        
    
    

    @Override
    public void end(boolean interrupted){

    }

    @Override
    public boolean isFinished(){
        return false;
    }
}