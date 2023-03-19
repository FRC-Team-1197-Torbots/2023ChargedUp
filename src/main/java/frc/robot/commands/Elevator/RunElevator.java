package frc.robot.commands.Elevator;

import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorArmConstants.STATE;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;


public class RunElevator extends CommandBase{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

    private Arm arm;
    private Claw claw;
    private Elevator elevator;
    private PIDController elevatorPID;

    private double currentPosition;
    private double targetPosition;
    private XboxController m_player1;
    
    //private STATE ElState;
    //private MoveElevator moveElevator;
    /*private enum MoveElevator{
        IDLE, ACTIVE
    }*/
    
    public static enum ElevatorState{
        UP,DOWN,IDLE
    }
    public static enum IntakeorScore{
        INTAKE, SCORE, IDLE
    }
    private ElevatorState ElState;
    private IntakeorScore m_IntakeorScore;
    private double target;
    private double elevatorOutput;
    private double maxOutput;
    public RunElevator(Arm armSystem, Claw clawSystem, Elevator elevatorSystem){
        ElState = ElevatorState.IDLE;
        m_IntakeorScore = IntakeorScore.INTAKE;
        arm = armSystem;
        claw = clawSystem;
        elevator = elevatorSystem;
        elevatorPID = new PIDController(0.001, 0, 0);
        //m_player1 = player1;
        addRequirements(elevatorSystem);
    }
    @Override
    public void initialize() {
        elevator.ResetEncoder();
    }
    @Override
    
    public void execute(){
        //System.out.println(elevator.GetElevatorPos());
        SmartDashboard.putNumber("Elevator Position", elevator.GetElevatorPos());
        elevatorOutput = elevatorPID.calculate(target - currentPosition);
        if(elevatorOutput > 0.25){
            elevatorOutput = 0.25;
        }
        elevator.SetElevatorSpeed(elevatorOutput);

        if(RobotContainer.player2_HoldButton.getPOV() == 0){
            if(m_IntakeorScore == IntakeorScore.IDLE){
                m_IntakeorScore = IntakeorScore.INTAKE;
            }
            else if(m_IntakeorScore == IntakeorScore.INTAKE){
                m_IntakeorScore = IntakeorScore.SCORE;
            }
            else if(m_IntakeorScore == IntakeorScore.SCORE){
                m_IntakeorScore = IntakeorScore.IDLE;
            }
        }
        if(RobotContainer.player2_HoldButton.getPOV() == 180){
            if(m_IntakeorScore == IntakeorScore.IDLE){
                m_IntakeorScore = IntakeorScore.SCORE;
            }
            else if(m_IntakeorScore == IntakeorScore.SCORE){
                m_IntakeorScore = IntakeorScore.INTAKE;
            }
            else if(m_IntakeorScore == IntakeorScore.INTAKE){
                m_IntakeorScore = IntakeorScore.IDLE;
            }
        }
        /* 
        if(RobotContainer.player2_HoldButton.getYButtonPressed()){
            ElState = ElevatorState.UP;
        }
        else if(RobotContainer.player2_HoldButton.getAButtonPressed()){
            ElState = ElevatorState.DOWN;
        }
        else{
            ElState = ElevatorState.IDLE;
        }
        
        
        */
        switch(m_IntakeorScore){
            case INTAKE:
                target = -5083;
                break;
            case SCORE:
                target = -9400;
                break;
            case IDLE://comment out if PID doesn't work
                target = 0;
                break;
        }
        
        SmartDashboard.putString("Elevator Mode", m_IntakeorScore.toString());

        /* 
        switch(ElState){
            case UP: 
            if(Math.abs(elevator.GetElevatorPos())>= Math.abs(target)){
                elevator.SetElevatorSpeed(0);}
            else{
                 elevator.SetElevatorSpeed(0.2);
            }
           
            //if(elevator.TopSwitch()){
            //    elevator.SetElevatorSpeed(0);}
            break;
            case DOWN: 
            if(Math.abs(elevator.GetElevatorPos())<=100){
                elevator.SetElevatorSpeed(0);
            }
            else{
                elevator.SetElevatorSpeed(-0.2);
                }
           
            //if(elevator.BotSwitch()){
            //    elevator.SetElevatorSpeed(0);}
            break;
            case IDLE:
            elevator.SetElevatorSpeed(0.02);
            break;
        }
        */
        
        /*switch(moveElevator){
            case IDLE:
            ElState = STATE.IDLE;
            break;
            case ACTIVE:
            ElState = STATE.TOP;
            break;
        }
        if(m_player1.getYButton()){
            moveElevator = MoveElevator.ACTIVE;
        }
        else{
            moveElevator = MoveElevator.IDLE;
        }

        switch(ElState){
            case IDLE: 
            targetPosition = ElevatorArmConstants.EL_IDLE_POSITION;
            //currentPosition = elevator.GetElevatorPos();
            elevator.SetElevatorSpeed(elevatorPID.calculate(ElevatorArmConstants.EL_IDLE_POSITION - currentPosition));
            /* 
            if(elevator.TopSwitch()){
                elevator.SetElevatorSpeed(0);}
            
            break;
            case INTAKE:
            targetPosition = ElevatorArmConstants.EL_IDLE_POSITION;
            break;
            case GRAB:
            targetPosition = ElevatorArmConstants.EL_IDLE_POSITION;
            break;
            case STORE:
            targetPosition = ElevatorArmConstants.EL_IDLE_POSITION;
            break;
            case MIDDLE: 
            targetPosition = ElevatorArmConstants.EL_IDLE_POSITION;
            //currentPosition = elevator.GetElevatorPos();
            elevator.SetElevatorSpeed(elevatorPID.calculate(ElevatorArmConstants.EL_MID_POSITION - currentPosition));
            /* 
            if(elevator.BotSwitch()){
                elevator.SetElevatorSpeed(0);}
            break;
            case TOP:
            targetPosition = ElevatorArmConstants.EL_IDLE_POSITION;
            //currentPosition = elevator.GetElevatorPos();
            elevator.SetElevatorSpeed(elevatorPID.calculate(ElevatorArmConstants.EL_TOP_POSITION - currentPosition));
            break;
        }*/

    }
    
    @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;

  }
        
    }//-9242 is around the max encoder value when elevator is fully extended

    

