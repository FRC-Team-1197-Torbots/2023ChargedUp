package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ElevatorArmConstants.STATE;
import frc.robot.Constants.ElevatorArmConstants.TARGET;
import frc.robot.subsystems.Elevator;

public class SetElevatorState extends CommandBase{
    private TARGET elTarget;
    private Elevator m_Elevator;
    public SetElevatorState(Elevator elevator, TARGET target){
        elTarget = target;
        m_Elevator = elevator;

    }
    @Override
    public void execute(){
        m_Elevator.setElevatorTarget(elTarget);
    }
    //hello can you see this

}
