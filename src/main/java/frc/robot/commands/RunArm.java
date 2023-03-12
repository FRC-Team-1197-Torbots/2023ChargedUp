package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.ElevatorArmConstants;
import edu.wpi.first.math.controller.PIDController;
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

    private double currentPosition;
    
    public static enum ElevatorLevel{
        BOTTOM, MIDDLE, TOP;
    }
    private ElevatorLevel ElState;

    public RunArm(Arm armSystem, Claw clawSystem, double speed){//Elevator elevatorSystem, double speed){
        arm = armSystem;
        claw = clawSystem;
        m_speed = speed;
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
        if(RobotContainer.player1_HoldButton.getPOV() == 0){
            arm.SetArmSpeed(m_speed);

        }
        else if(RobotContainer.player1_HoldButton.getPOV() == 180){
            arm.SetArmSpeed(-m_speed);
        }
        else{
            arm.SetArmSpeed(0);
        }

    }
        
    public void setSpeed(double speed){
        m_speed = speed;
    }
    }//-9242 is around the max encoder value when elevator is fully extended

    

