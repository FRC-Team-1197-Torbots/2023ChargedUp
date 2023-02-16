package frc.robot.commands;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3.RawColor;

import edu.wpi.first.wpilibj.DigitalInput;
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

        
        ColorMatch m_colorMatcher = new ColorMatch();

        Color kBlueTarget = new Color(0.269775, 0.474121, 0.365234);
        Color kYellowTarget = new Color(0.304, 0.487, 0.116943);

    /**
     * Run the color match algorithm on our detected color
     */
    String colorString;
    ColorMatchResult match = m_colorMatcher.matchClosestColor(color);


    if(hopperSubsystem.breakBeam.get()) {
      if (color.blue >= 0.365234) {
        colorString = "Cube";
        System.out.println("i saw a cube!");
      } else {
        colorString = "Cone";
        System.out.println("i saw a cone!");
    }}
    else{
      colorString = "None";
      System.out.println("i saw a nothing!");
    }


    SmartDashboard.putBoolean("Limit Switch",hopperSubsystem.limitSwitch.get());
    SmartDashboard.putBoolean("Breakbeam",hopperSubsystem.breakBeam.get());

    SmartDashboard.putString("Object", colorString);


    }

}

