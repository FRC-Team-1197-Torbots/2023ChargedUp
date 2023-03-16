package frc.robot.subsystems;

import org.photonvision.PhotonCamera;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeHopperConstants;
public class Hopper extends SubsystemBase {
    public static DigitalInput limitSwitch;
    public static DigitalInput breakBeam;
    public static PhotonCamera hoppercam;
    public static I2C.Port i2cPort = I2C.Port.kOnboard;
    public Timer timer;
    //public CANSparkMax HopperMotor = new CANSparkMax(IntakeHopperConstants.HopperID, MotorType.kBrushless);


    public Hopper() {

        //I2C arduino = new I2C(i2cPort, 4);
        breakBeam = new DigitalInput(5);
        hoppercam = new PhotonCamera("1197HopperCam");
        

    }
}
