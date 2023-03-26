package frc.robot.commands.Pneumatics;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Pneumatics;

public class RunCompressor extends CommandBase{
    private Pneumatics m_Pneumatics;

    public RunCompressor(Pneumatics pneumatics){
        m_Pneumatics = pneumatics;
        addRequirements(m_Pneumatics);
        m_Pneumatics.startCompressor();
    }

    @Override
    public void execute(){
        //System.out.println("Running Compressor");        
    }

    
    @Override
    public void end(boolean interrupted){

    }

    @Override
    public boolean isFinished(){
        return false;
    }
    
}
