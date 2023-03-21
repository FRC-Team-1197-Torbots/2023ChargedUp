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
    private boolean clawState;
    public Claw(){
        clawSolenoid = new Solenoid(PneumaticsModuleType.REVPH, 2);
        clawMotor = new CANSparkMax(ElevatorArmConstants.ClawID, MotorType.kBrushless);
    }
    public void SetClawSpeed(double speed){
        clawMotor.set(speed);
    }
    public void dropClaw(){
        //clawSolenoid.set(!clawSolenoid.get());
        clawSolenoid.toggle();
        System.out.println("Claw scolenoid: " + clawSolenoid.get());
    }
    public boolean getSolenoidValue(){
        return clawSolenoid.get();
    }
    public double getCurrent(){
        return clawMotor.getOutputCurrent();
    }
}
