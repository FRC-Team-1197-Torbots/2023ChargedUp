package frc.robot.commands.Intake;

import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.ElevatorArmConstants.GamePiece;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
public class IntakeGamePiece extends CommandBase {

    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Intake intakeSystem;
    
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
        m_IntakeState = IntakeState.UP;
        addRequirements(subsystem);

    }

    @Override
    public void initialize(){}

    @Override
    public void execute(){
        intakeSystem.SetSolenoid(true);
        intakeSystem.SetRollerSpeed(0.5, 0.6);
        //System.out.println("Intake Down");
        
        /* 
        switch(m_IntakeState){
            case UP:
                System.out.println("Intake up");
                state = true;
                intakeSystem.SetRollerSpeed(0, 0);
                Timer.delay(0.5);
                intakeSystem.SetSolenoid(state);
                //System.out.println("intake running");
                break;
            case DOWN:
                System.out.println("Intake down");
                state = false;
                intakeSystem.SetSolenoid(state);
                Timer.delay(0.5);
                //System.out.println("intake off");
                //intakeSystem.SetRollerSpeed(0.5, 0.6);//Cube speed: top = 0.5, bot = 0.6
                 
                break;
            */
        }


    @Override
    public void end(boolean interrupted){
        //System.out.println("Intake Up");
        intakeSystem.SetSolenoid(false);

    }

    @Override
    public boolean isFinished(){
        return !RobotContainer.player1.rightBumper().getAsBoolean();
    }

}