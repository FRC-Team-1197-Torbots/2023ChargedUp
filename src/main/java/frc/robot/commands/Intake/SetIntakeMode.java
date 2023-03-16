package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ElevatorArmConstants.GamePiece;
import frc.robot.subsystems.Intake;

public class SetIntakeMode extends CommandBase{
    private GamePiece m_gamePiece;
    private Intake m_Intake;
    public SetIntakeMode(Intake intake, GamePiece gamePiece){
        m_gamePiece = gamePiece;
        m_Intake = intake;
    }

    public GamePiece getGamePiece(){
        return m_gamePiece;
    }

    @Override
    public void execute(){
        m_Intake.setGamePiece(m_gamePiece);
    }
    
}
