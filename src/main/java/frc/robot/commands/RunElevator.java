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
    private double speed;
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
    public void execute(){
        if(RobotContainer.player1.getPOV() == 0){
            ElState = ElevatorState.UP;
        }
        else if(RobotContainer.player1.getPOV() == 180){
            ElState = ElevatorState.DOWN;
        }
        else{
            ElState = ElevatorState.IDLE;
        }

        switch(ElState){
            case UP: 
            elevator.SetElevatorSpeed(speed);
            if(elevator.TopSwitch()){
                elevator.SetElevatorSpeed(0);}
            break;
            case DOWN: 
            elevator.SetElevatorSpeed(-speed);
            if(elevator.BotSwitch()){
                elevator.SetElevatorSpeed(0);}
            break;
            case IDLE:
            elevator.SetElevatorSpeed(0);
            break;
        }

            }
        
    }

    

