package frc.robot.commands.Arm;

import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.ElevatorArmConstants;
import frc.robot.Constants.ElevatorArmConstants.MoveElevatorArm;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;

public class RunArm extends CommandBase{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

    private Arm arm;
    private Claw claw;
    private Elevator elevator;
    private PIDController elevatorPID;
    private double m_speed;
    private XboxController m_player1;

    private double currentPosition;
    private MoveElevatorArm moveArm;
    

    public RunArm(Arm armSystem, Claw clawSystem, XboxController player1){//Elevator elevatorSystem, double speed){
        arm = armSystem;
        claw = clawSystem;
        m_player1 = player1;
        //elevator = elevatorSystem;
        elevatorPID = new PIDController(0, 0, 0);
        addRequirements(armSystem, clawSystem);//, //elevatorSystem);
    }
    @Override
    public void initialize() {
        //elevator.ResetEncoder();
    }
    @Override
    public void execute(){
        //System.out.println("Encoder value " + elevator.GetEncoderValue());
        //System.out.println("Encoder rate " + elevator.GetEncoderRate());
        switch(moveArm){
            case IDLE:
            break;
            case ACTIVE:
            break;
        }
        if(m_player1.getYButton()){
            moveArm = MoveElevatorArm.ACTIVE;
        }
        else{
            moveArm = MoveElevatorArm.IDLE;
        }
        

    }
        
    public void setSpeed(double speed){
        m_speed = speed;
    }
    }//-9242 is around the max encoder value when elevator is fully extended

    

