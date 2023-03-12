package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorArmConstants;
import frc.robot.Constants.ElevatorArmConstants.STATE;

public class Elevator extends SubsystemBase{
    private CANSparkMax elMotor1;
    private CANSparkMax elMotor2;
    private DigitalInput botlimitSwitch;
    private DigitalInput toplimitSwitch;
    private Encoder elEncoder;
    private STATE m_Elstate;

    public Elevator(){
        elMotor1 = new CANSparkMax(ElevatorArmConstants.ElMotor1ID, MotorType.kBrushless);
        elMotor2 = new CANSparkMax(ElevatorArmConstants.ElMotor2ID, MotorType.kBrushless);
        //botlimitSwitch = new DigitalInput(ElevatorArmConstants.limitSwitch1Port);
        //toplimitSwitch = new DigitalInput(ElevatorArmConstants.limitSwitch2Port);
        elEncoder = new Encoder(0, 1);//Input correct channels later
        ResetEncoder();
    }
    


    public void SetElevatorSpeed(double speed){
        elMotor1.set(speed);
        elMotor2.set(speed);

    }

    public void setElevatorState(STATE state){
        m_Elstate = state;
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Encoder Value", GetEncoderValue());
    }
    /* 
    public boolean TopSwitch(){
        return toplimitSwitch.get();
    }
    public boolean BotSwitch(){
        return botlimitSwitch.get();
    }
    */
    public double GetEncoderValue(){
        return elEncoder.get();
    }
    public double GetEncoderRate(){
        return elEncoder.getRate();
    }
    public void ResetEncoder(){
        elEncoder.reset();
    }



}