package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorArmConstants;

public class Claw extends SubsystemBase{
    private Solenoid clawSolenoid;
    private CANSparkMax clawMotor;
    public Claw(){
        clawSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 1);
        //clawMotor = new CANSparkMax(ElevatorArmConstants.ClawID, MotorType.kBrushless);
    }
    
}
