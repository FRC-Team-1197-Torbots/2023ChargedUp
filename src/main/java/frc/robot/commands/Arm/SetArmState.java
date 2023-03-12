package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ElevatorArmConstants.STATE;
import frc.robot.subsystems.Arm;

public class SetArmState extends CommandBase{
    private STATE m_ArmState;
    private Arm m_arm;
    public SetArmState(Arm arm, STATE armState){
        m_arm = arm;
        m_ArmState = armState;
    }
    
}
