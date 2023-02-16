package frc.robot.subsystems;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hopper extends SubsystemBase {
    public static ColorSensorV3 colorsensor;
    public static DigitalInput limitSwitch;
    public static DigitalInput breakBeam;

    private final I2C.Port i2cPort = I2C.Port.kOnboard
    ;


    public Hopper() {

        colorsensor = new ColorSensorV3(i2cPort);
        limitSwitch = new DigitalInput(4);
        breakBeam = new DigitalInput(5);

    }
}
