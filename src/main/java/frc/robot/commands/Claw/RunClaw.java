package frc.robot.commands.Claw;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;
import frc.robot.RobotContainer;
public class RunClaw extends CommandBase{
    private Claw m_Claw;
    private double ClawIntakeSpeed = 0.3;
    private double ClawEjectSpeed=-0.2;

    public RunClaw(Claw claw){
        m_Claw = claw;
        addRequirements(claw);
    }

    @Override
    public void execute(){
        if(RobotContainer.player1_HoldButton.getAButtonPressed()){
            m_Claw.SetClawSpeed(ClawIntakeSpeed);
            
        }
        if(RobotContainer.player1_HoldButton.getBButtonPressed()){
            m_Claw.SetClawSpeed(ClawEjectSpeed);
            
        }
    }
    
}
