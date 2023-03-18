package frc.robot.commands.Claw;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;

public class RunClaw extends CommandBase{
    private Claw m_Claw;
    public RunClaw(Claw claw){
        m_Claw = claw;
    }

    @Override
    public void execute(){
        m_Claw.dropClaw();
    }
    
}
