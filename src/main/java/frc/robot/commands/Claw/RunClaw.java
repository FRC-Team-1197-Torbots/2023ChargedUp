package frc.robot.commands.Claw;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;
import frc.robot.RobotContainer;
public class RunClaw extends CommandBase{
    private Claw m_Claw;
    private double speed;
    private double ConeIntakeSpeed = 0.3;
    private double CubeIntakeSpeed = 0.5;
    private double CubeEjectSpeed = -0.4;
    private double IdleSpeed = 0.02;
    private double MaxClawCurrent = 10;
    //private double ConeIntakeSpeed = 0.5;
    private enum gamePieceState{
        CUBE, CONE, IDLE, EJECT_CUBE
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
        SmartDashboard.putNumber("Claw Current", m_Claw.getCurrent());
        //m_Claw.SetClawSpeed(speed);
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
        if(RobotContainer.player1_HoldButton.getBButtonPressed()){
            //System.out.println("drop claw");
            m_Claw.dropClaw();}

        if(RobotContainer.player1_HoldButton.getRightBumper()){
            m_PieceState = gamePieceState.CONE;
        }
        else if(RobotContainer.player1_HoldButton.getLeftBumper()){
            m_PieceState = gamePieceState.CUBE;
        }
        else if (RobotContainer.player1_HoldButton.getRightTriggerAxis()>0.2)
            m_PieceState = gamePieceState.EJECT_CUBE;
        else{
            m_PieceState = gamePieceState.IDLE;
        }


        if(m_Claw.getCurrent() > MaxClawCurrent){
            m_Claw.SetClawSpeed(0);
        }
        else{        
        switch(m_PieceState){
            case CUBE:
                //speed = CubeIntakeSpeed;
                m_Claw.SetClawSpeed(CubeIntakeSpeed);
                break;
            case EJECT_CUBE:
                m_Claw.SetClawSpeed(-CubeIntakeSpeed);
                break;
            case CONE:
                //speed = ConeIntakeSpeed;
                m_Claw.SetClawSpeed(ConeIntakeSpeed);
                /*if(!m_Claw.getSolenoidValue()){ 
                        m_Claw.SetClawSpeed(speed);
                        //m_PieceState = gamePieceState.CUBE;
                    }
                else{
                        m_Claw.SetClawSpeed(speed);
                        //m_PieceState = gamePieceState.CONE;
                    }*/
                
                break;
                case IDLE:
                    m_Claw.SetClawSpeed(0);
                    break;
            }}
        }
    }