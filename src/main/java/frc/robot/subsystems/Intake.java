package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Solenoid;

//Right Bumber cone,Left Bumber cube
public class Intake extends SubsystemBase{
    private Solenoid Intakeout;
    private CANSparkMax Roller;
    private Encoder IntakeEncoder;
    public Intake(){
        Intakeout = new Solenoid(null, 0);
        Roller = new CANSparkMax(0, MotorType.kBrushless);
        IntakeEncoder = new Encoder(0, 0);
        IntakeEncoder.reset();
    }
    
    public double GetEncoderValue(){
        return IntakeEncoder.get();
    }
    public void SetRollerSpeed(double speed){
        Roller.set(speed);
    }
    public void SetSolenoid(boolean value){
        Intakeout.set(value);
    }

}
