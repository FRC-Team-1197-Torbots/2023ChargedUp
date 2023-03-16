package frc.robot.commands.Pneumatics;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Pneumatics;

public class RunCompressor extends CommandBase{
    private Pneumatics m_Pneumatics;

    public RunCompressor(Pneumatics pneumatics){
        m_Pneumatics = pneumatics;
        addRequirements(m_Pneumatics);

    }

    @Override
    public void execute(){
        m_Pneumatics.startCompressor();
    }
    
}
