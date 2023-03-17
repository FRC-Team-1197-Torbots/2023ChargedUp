package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pneumatics extends SubsystemBase{
    private Compressor m_Compressor;
    public Pneumatics(){
        m_Compressor = new Compressor(PneumaticsModuleType.REVPH);
    }

    public void startCompressor(){
        System.out.println("compressor enabled");
        m_Compressor.enableDigital();
    }
    
}
