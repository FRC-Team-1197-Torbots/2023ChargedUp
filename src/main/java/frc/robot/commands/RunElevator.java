package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.ElevatorArmConstants;
import edu.wpi.first.math.controller.PIDController;
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
    
    public static enum ElevatorLevel{
        BOTTOM, MIDDLE, TOP;
    }
    private ElevatorLevel ElState;

    public RunElevator(Arm armSystem, Claw clawSystem, Elevator elevatorSystem, ElevatorLevel state){
        ElState = state;
        arm = armSystem;
        claw = clawSystem;
        elevator = elevatorSystem;
        elevatorPID = new PIDController(0, 0, 0);
        addRequirements(armSystem, clawSystem, elevatorSystem);
    }
    @Override
    public void initialize() {
        elevator.ResetEncoder();
    }
    @Override
    
    public void execute(){
        //System.out.println("Encoder value " + elevator.GetEncoderValue());
        //System.out.println("Encoder rate " + elevator.GetEncoderRate());

        switch(ElState){
            case BOTTOM: 
            currentPosition = elevator.GetEncoderValue();
            elevator.SetElevatorSpeed(elevatorPID.calculate(ElevatorArmConstants.BOTTOM_POSITION - currentPosition));
            /* 
            if(elevator.TopSwitch()){
                elevator.SetElevatorSpeed(0);}
            */
            break;
            case MIDDLE: 
            currentPosition = elevator.GetEncoderValue();
            elevator.SetElevatorSpeed(elevatorPID.calculate(ElevatorArmConstants.MID_POSITION - currentPosition));
            /* 
            if(elevator.BotSwitch()){
                elevator.SetElevatorSpeed(0);}*/
            break;
            
            case TOP:
            currentPosition = elevator.GetEncoderValue();
            elevator.SetElevatorSpeed(elevatorPID.calculate(ElevatorArmConstants.TOP_POSITION - currentPosition));
            break;
        }

            }
        
    }//-9242 is around the max encoder value when elevator is fully extended

    

