package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ElevatorArmConstants.STATE;
import frc.robot.Constants.ElevatorArmConstants.TARGET;
import frc.robot.subsystems.Arm;

public class SetArmState extends CommandBase{
    private TARGET armTarget;
    private Arm m_Arm;
    public SetArmState(Arm arm, TARGET target){
        armTarget = target;
        m_Arm = arm;

    }
    @Override
    public void execute(){
        m_Arm.setArmTarget(armTarget);
    }
}
