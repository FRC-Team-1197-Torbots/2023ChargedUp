package frc.robot.commands.Elevator;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ElevatorArmConstants;
import frc.robot.Constants.ElevatorArmConstants.GamePiece;
import frc.robot.Constants.ElevatorArmConstants.STATE;
import frc.robot.Constants.ElevatorArmConstants.TARGET;
import frc.robot.subsystems.Elevator;

public class AutoRunElevator extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Elevator m_Elevator;
    private TARGET m_target;
    private GamePiece m_GamePiece;
    private STATE m_State;
    private double m_targetPoint;
    public AutoRunElevator(Elevator elevator, TARGET target, GamePiece gamePiece, STATE state){
        m_Elevator = elevator;
        m_target = target;
        m_GamePiece = gamePiece;
        m_State = state;
        addRequirements(elevator);
    }
    public void initialize(){
        
    }
    @Override
    public void execute(){

    }
    @Override
    public void end(boolean interrupted){

    }
    @Override
    public boolean isFinished(){
        return (Math.abs(m_Elevator.GetElevatorPos() - m_targetPoint) < 35);
    }
}