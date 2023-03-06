package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;

public class RunElevator extends CommandBase{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

    private Arm arm;
    private Claw claw;
    private Elevator elevator;
    
    public static enum ElevatorState{
        UP, DOWN, IDLE;
    }
    private ElevatorState ElState;
    public RunElevator(Arm armSystem, Claw clawSystem, Elevator elevatorSystem){
        ElState = ElevatorState.IDLE;
        arm = armSystem;
        claw = clawSystem;
        elevator = elevatorSystem;
        addRequirements(armSystem, clawSystem, elevatorSystem);
    }
    @Override
    public void initialize() {
        elevator.ResetEncoder();
    }
    @Override
    
    public void execute(){
        System.out.println("Encoder value " + elevator.GetEncoderValue());
        //System.out.println("Encoder rate " + elevator.GetEncoderRate());
        if(RobotContainer.player1.getXButton()){
            ElState = ElevatorState.UP;
            
        }
        else if(RobotContainer.player1.getBButton()){
            ElState = ElevatorState.DOWN;
        }
        else{
            ElState = ElevatorState.IDLE;
        }

        switch(ElState){
            case UP: 
            elevator.SetElevatorSpeed(0.25);
            /* 
            if(elevator.TopSwitch()){
                elevator.SetElevatorSpeed(0);}
            */
            break;
            case DOWN: 
            elevator.SetElevatorSpeed(-0.25);
            /* 
            if(elevator.BotSwitch()){
                elevator.SetElevatorSpeed(0);}*/
            break;
            
            case IDLE:
            elevator.SetElevatorSpeed(0.05);
            break;
        }

            }
        
    }//-9242 is around the max encoder value when elevator is fully extended

    

