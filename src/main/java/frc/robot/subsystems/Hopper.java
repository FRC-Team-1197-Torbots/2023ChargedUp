package frc.robot.subsystems;

import org.photonvision.PhotonCamera;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.util.Color;
public class Hopper extends SubsystemBase {
    public static DigitalInput limitSwitch;
    public static DigitalInput breakBeam;
    public static PhotonCamera hoppercam;
    public static I2C.Port i2cPort = I2C.Port.kOnboard;
    public Timer timer;
    public static ColorSensorV3 colorsensor;


    public Hopper() {

        //I2C arduino = new I2C(i2cPort, 4);
        breakBeam = new DigitalInput(5);
        hoppercam = new PhotonCamera("1197HopperCam");
        colorsensor = new ColorSensorV3(i2cPort);


    }
}
