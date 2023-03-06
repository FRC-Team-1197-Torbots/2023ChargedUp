package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorArmConstants;

public class Arm extends SubsystemBase{
    private CANSparkMax armMotor;
    private DigitalInput armSwitch1;
    private DigitalInput armSwitch2;
    public Arm(){
        //armMotor = new CANSparkMax(ElevatorArmConstants.ArmID, MotorType.kBrushless);
        armSwitch1 = new DigitalInput(ElevatorArmConstants.armSwitch1Port);
        armSwitch2 = new DigitalInput(ElevatorArmConstants.armSwitch2Port);
    }

    
}
