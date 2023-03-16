package frc.robot.commands.Drive;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class SetLED extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

    byte[] m_ledsignal = {0x00};

    @Override
    public void initialize(){
    }
    
    public SetLED(int ledsignal) {
        if(ledsignal == 1){ //cone
            m_ledsignal[0] = 0x01;
        }
        else if(ledsignal == 2){//cube
            m_ledsignal[0] = 0x02;
        }
	}

    @Override
    public void execute(){
        
        int SERVICE_PORT = 8888;

    try{
      /* Instantiate client socket. 
      No need to bind to a specific port */
      DatagramSocket clientSocket = new DatagramSocket();
      
      // Get the IP address of the server
      InetAddress IPAddress = InetAddress.getByName("10.11.97.77");
      
      // Creating a UDP packet 
      DatagramPacket sendingPacket = new DatagramPacket(m_ledsignal, m_ledsignal.length,IPAddress, SERVICE_PORT);
      
      // sending UDP packet to the server
      clientSocket.send(sendingPacket);
      
      // Closing the socket connection with the server
      clientSocket.close();
    }
      catch(IOException e) {
      e.printStackTrace();
    }

    }
}
