package frc.robot.commands;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3.RawColor;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hopper;

public class ColorSensorTest extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Hopper hopperSubsystem;

    public ColorSensorTest(Hopper subsystem) {
        hopperSubsystem = subsystem;

        addRequirements(subsystem);

    }
    @Override
    public void initialize() {}

    @Override
    public void execute() {
        Color color = hopperSubsystem.colorsensor.getColor();
        boolean connected = hopperSubsystem.colorsensor.isConnected();
        //int red = rawcolor.red;
        //int green = rawcolor.green;
        //int blue = rawcolor.blue;
        //System.out.println(color.toString());//"Red: " + red + ", Green: " + green + ", Blue: " + blue);
        //System.out.println(connected);

        /**
         * The sensor returns a raw IR value of the infrared light detected.
         */
        double IR = hopperSubsystem.colorsensor.getIR();
    
        /**
         * Open Smart Dashboard or Shuffleboard to see the color detected by the 
         * sensor.
         */
        SmartDashboard.putNumber("Red", color.red);
        SmartDashboard.putNumber("Green", color.green);
        SmartDashboard.putNumber("Blue", color.blue);
        SmartDashboard.putNumber("IR", IR);
        SmartDashboard.putString("color",color.toString());
        System.out.println(color.toString());

        
        ColorMatch m_colorMatcher = new ColorMatch();

        Color kBlueTarget = new Color(0.277344, 0.479492, 0.243408);
        Color kYellowTarget = new Color(0.304443, 0.487549, 0.208496);

    /**
     * Run the color match algorithm on our detected color
     */
    String colorString;
    ColorMatchResult match = m_colorMatcher.matchClosestColor(color);

    if (match.color == kBlueTarget) {
      colorString = "Cube";
    } else if (match.color == kYellowTarget) {
      colorString = "Cone";
    } else {
      colorString = "None";
    }

    SmartDashboard.putString("Object", colorString);


    }

}

