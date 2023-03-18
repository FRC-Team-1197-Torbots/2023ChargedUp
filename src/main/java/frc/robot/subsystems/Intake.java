package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.Constants.IntakeHopperConstants;
import frc.robot.Constants.ElevatorArmConstants.GamePiece;

//Right Bumber cone,Left Bumber cube
public class Intake extends SubsystemBase{
    private Solenoid Intakeout;
    private CANSparkMax RollerBottom;
    private CANSparkMax RollerTop;
    private Encoder IntakeEncoder;
    private GamePiece m_gamePiece;
    public Intake(){
        Intakeout = new Solenoid(PneumaticsModuleType.REVPH, 0);
        RollerBottom = new CANSparkMax(IntakeHopperConstants.Intake1ID, MotorType.kBrushless);
        RollerTop = new CANSparkMax(IntakeHopperConstants.Intake2ID, MotorType.kBrushless);
        //IntakeEncoder = new Encoder(0, 0);
        //IntakeEncoder.reset();
    }
    public void setGamePiece(GamePiece gamePiece){
        m_gamePiece = gamePiece;
    }
    /* 
    public double GetEncoderValue(){
        return IntakeEncoder.get();
    }
    */
    public void SetRollerSpeed(double speed){
        RollerBottom.set(speed);
        RollerTop.set(-speed);
    }
    
    public void SetSolenoid(boolean value){
        Intakeout.set(value);
    }
    

}
