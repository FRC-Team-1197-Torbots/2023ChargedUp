package frc.robot.commands.Hopper;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.SocketException;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

//import com.revrobotics.ColorSensorV3.RawColor;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hopper;

public class Spindex extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Hopper hopperSubsystem;
    public Spindex(Hopper subsystem) {
        hopperSubsystem = subsystem;
        
        addRequirements(subsystem);
    }
    @Override
    public void initialize() {}

    @Override
    public void execute() {
        //Color color = hopperSubsystem.colorsensor.getColor();
        //boolean connected = hopperSubsystem.colorsensor.isConnected();

        
        //int red = rawcolor.red;
        //int green = rawcolor.green;
        //int blue = rawcolor.blue;
        //System.out.println(color.toString());//"Red: " + red + ", Green: " + green + ", Blue: " + blue);
        //System.out.println(connected);

        /**
         * The sensor returns a raw IR value of the infrared light detected.
         */
        //double IR = hopperSubsystem.colorsensor.getIR();
    
        /**
         * Open Smart Dashboard or Shuffleboard to see the color detected by the 
         * sensor.
         */
        /*SmartDashboard.putNumber("Red", color.red);
        SmartDashboard.putNumber("Green", color.green);
        SmartDashboard.putNumber("Blue", color.blue);
        SmartDashboard.putNumber("IR", IR);
        SmartDashboard.putString("color",color.toString());
        System.out.println(color.toString());
        */

        hopperSubsystem.hoppercam.setPipelineIndex(0);

        //byte[] ledsignal = {0x00}; //
        /*
        if cone state pressed:
        spin, have limelight look for targets
        if target detected, check for skew value
        skew range: -15 to +15 degrees
        if skew is within range, stop spinning
         */

        //get what limelight sees. if a target is detected, it's a cone
        var camresult = hopperSubsystem.hoppercam.getLatestResult();
        if (camresult.hasTargets()){
          PhotonTrackedTarget camtarget = camresult.getBestTarget();
          double skew = camtarget.getSkew();
          SmartDashboard.putNumber("skew", skew);
          if(Math.abs(skew) <= 20){
            //stop motor
          }
          
          //ledsignal[0] = 0x01;
        }
        // if no target from limelight but breakbeam sees something, it's a cube
        else if(hopperSubsystem.breakBeam.get()){
            SmartDashboard.putString("status","Cube found");
            //ledsignal[0] = 0x02;
        }
        else{
          SmartDashboard.putString("status", "No object found");
        }

        /*if (color.blue >= 0.365234) {
          colorString = "Cube";
        }
        else {
          colorString = "Cone";
        }
        */
        /*if(camtarget != null){
          double skew = camtarget.getSkew();
          SmartDashboard.putNumber("skew", skew);
          SmartDashboard.putString("status","Cone found, rotate " + -skew + " degrees");
        }
        else if(hopperSubsystem.breakBeam.get()){
          var camresult1 = hopperSubsystem.hoppercam.getLatestResult();
          boolean camtarget1 = camresult1.hasTargets();
          if(colorString == "Cone"){
            hopperSubsystem.hoppercam.setPipelineIndex(1);
            if(camtarget1){
              SmartDashboard.putString("status","Cone found");
            }
            else{
              SmartDashboard.putString("status","Cone found, upright");
              //activate piston
            }
          }
          else if(colorString == "Cube"){
            hopperSubsystem.hoppercam.setPipelineIndex(2);
            if(camtarget1){
              SmartDashboard.putString("status","Cube found");
          }
            } 
          }
          */
          /*hopperSubsystem.hoppercam.setPipelineIndex(1);
          Timer.delay(2);
          var camresult1 = hopperSubsystem.hoppercam.getLatestResult();
          boolean camtarget1 = camresult1.hasTargets();
          if(camtarget1){
            SmartDashboard.putString("status","Cube found");
          }
          else{
            SmartDashboard.putString("status","Cone found, upright");
          }
        }
        else{
          SmartDashboard.putString("status", "No object found");
        }  */

        /*        
        ColorMatch m_colorMatcher = new ColorMatch();

        Color kBlueTarget = new Color(0.269775, 0.474121, 0.365234);
        Color kYellowTarget = new Color(0.304, 0.487, 0.116943); */
        
    /**
     * Run the color match algorithm on our detected color
     */
    //ColorMatchResult match = m_colorMatcher.matchClosestColor(color);


/*    if(hopperSubsystem.breakBeam.get()) {
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
*/
    SmartDashboard.putBoolean("Breakbeam",hopperSubsystem.breakBeam.get());

    /*int SERVICE_PORT = 8888;

    try{
      //Instantiate client socket. 
      //No need to bind to a specific port
      DatagramSocket clientSocket = new DatagramSocket();
      
      // Get the IP address of the server
      InetAddress IPAddress = InetAddress.getByName("10.11.97.77");
      
      // Creating a UDP packet 
      DatagramPacket sendingPacket = new DatagramPacket(ledsignal,ledsignal.length,IPAddress, SERVICE_PORT);
      
      // sending UDP packet to the server
      clientSocket.send(sendingPacket);
      
      // Closing the socket connection with the server
      clientSocket.close();
    }
      catch(IOException e) {
      e.printStackTrace();
    }
    */

    //SmartDashboard.putString("Object", colorString);
    }

}