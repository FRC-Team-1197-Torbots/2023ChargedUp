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
import frc.robot.subsystems.Intake;
import frc.robot.commands.Arm.RunArm;
import frc.robot.commands.Arm.RunArm.MoveArm;

public class RunElevator extends CommandBase{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

    private Arm arm;
    private Claw claw;
    private Elevator elevator;
    private Intake m_intake;
    private PIDController elevatorPID;
    private RunArm Arm;
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
    public static enum LEVEL{
        BOTTOM, MIDDLE, TOP
    }
    //private ElevatorState ElState;
    private IntakeorScore m_IntakeorScore;
    private LEVEL m_level;
    private double target;
    private double elevatorOutput;
    private double maxOutput;
    private boolean targetReached;
    
    public RunElevator(Arm armSystem, Claw clawSystem, Elevator elevatorSystem, Intake intakeSystem, LEVEL level){
        //ElState = ElevatorState.IDLE;
        m_IntakeorScore = IntakeorScore.IDLE;
        m_level = level;
        m_intake = intakeSystem;
        arm = armSystem;
        claw = clawSystem;
        elevator = elevatorSystem;
        elevatorPID = new PIDController(0.000001, 0, 0.0005);
        //m_player1 = player1;
        addRequirements(elevatorSystem);
    }
    @Override
    public void initialize() {
        
    }
    @Override
    
    public void execute(){//remember to commment out runarm before testing elevator pid
        currentPosition = elevator.GetElevatorPos();
        //System.out.println("Current Position: "+ currentPosition + " target Position: " + target + " elevator Output: " + elevatorOutput);
        //SmartDashboard.putNumber("Elevator Position", currentPosition);
        elevatorOutput = elevatorPID.calculate(target - currentPosition);
        // if(elevatorOutput > 0.25){
        //     elevatorOutput = 0.25;
        // }
        elevator.SetElevatorSpeed(elevatorOutput);
        if (Math.abs(target-currentPosition)<=20){
            targetReached=true;
        }
        else{
            targetReached=false;
        }
        SmartDashboard.putBoolean("Elevator Target Reached?", targetReached);
        
        switch(m_level){
            case BOTTOM:
                System.out.println("Bottom");
                m_intake.SetSolenoid(true);
                target = 5083;
                break;
            case MIDDLE:
                System.out.println("Middle");
                m_intake.SetSolenoid(true);
                target = 9400;
                break;
            case TOP://comment out if PID doesn't work
                System.out.println("Top");
                m_intake.SetSolenoid(true);
                target = 0;
                break;
        }
        
        SmartDashboard.putString("Elevator Mode", m_IntakeorScore.toString());

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

    

