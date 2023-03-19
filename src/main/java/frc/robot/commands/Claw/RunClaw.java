package frc.robot.commands.Claw;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;
import frc.robot.RobotContainer;
public class RunClaw extends CommandBase{
    private Claw m_Claw;
    private double speed;
    private double CubeIntakeSpeed = 0.5;
    private double CubeEjectSpeed = -0.4;
    private double IdleSpeed = 0.02;
    //private double ConeIntakeSpeed = 0.5;
    private enum gamePieceState{
        CUBE, CONE, IDLE
    }

    private gamePieceState m_PieceState;

    public RunClaw(Claw claw){
        m_Claw = claw;
        speed = IdleSpeed;
        m_PieceState = gamePieceState.CONE;
        addRequirements(claw);

    }

    @Override
    public void execute(){
        SmartDashboard.putBoolean("Claw Open?", m_Claw.getSolenoidValue());
        SmartDashboard.putString("Game Piece", m_PieceState.toString());
        m_Claw.SetClawSpeed(speed);
        //System.out.println("execute");
        /* 
        if(RobotContainer.player1_HoldButton.getRightBumperPressed()){
            if(m_PieceState == gamePieceState.CONE){
                m_PieceState = gamePieceState.CUBE;
            }
            else{
                m_PieceState = gamePieceState.CONE;
            }

        }
        */
        if(RobotContainer.player1_HoldButton.getRightBumper()){
            m_PieceState = gamePieceState.CONE;
        }
        if(RobotContainer.player1_HoldButton.getLeftBumper()){
            m_PieceState = gamePieceState.CUBE;
        }
        else{
            m_PieceState = gamePieceState.IDLE;
        }
        
        switch(m_PieceState){
            case CUBE:
                speed = CubeIntakeSpeed;
                break;
            case CONE:
                if(RobotContainer.player1_HoldButton.getBButtonPressed()){
                    //System.out.println("drop claw");
                    m_Claw.dropClaw();
                    speed = IdleSpeed;
                    //m_Claw.SetClawSpeed(speed);
                    if(!m_Claw.getSolenoidValue()){ 
                        m_Claw.SetClawSpeed(speed);
                        //m_PieceState = gamePieceState.CUBE;
                    }
                    else{
                        m_Claw.SetClawSpeed(speed);
                        //m_PieceState = gamePieceState.CONE;
                    }
                }
                break;
        }
    }
}
        // if(RobotContainer.player1_HoldButton.getBButtonPressed()){
        //     
            

    
    

