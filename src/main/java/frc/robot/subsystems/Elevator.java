package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorArmConstants;

public class Elevator extends SubsystemBase{
    private CANSparkMax elMotor1;
    private CANSparkMax elMotor2;
    private DigitalInput limitSwitch1;
    private DigitalInput limitSwitch2;

    public Elevator(){
        elMotor1 = new CANSparkMax(ElevatorArmConstants.ElMotor1ID, MotorType.kBrushless);
        elMotor2 = new CANSparkMax(ElevatorArmConstants.ElMotor2ID, MotorType.kBrushless);
        limitSwitch1 = new DigitalInput(ElevatorArmConstants.limitSwitch1Port);
        limitSwitch2 = new DigitalInput(ElevatorArmConstants.limitSwitch2Port);
    }
    
}
