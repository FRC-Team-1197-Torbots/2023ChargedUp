package frc.robot.commands;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;

import frc.robot.commands.Intake.IntakeGamePiece;
import frc.robot.commands.Intake.IntakeGamePiece.HopperPiece;

public class LED {

    byte[] ledsignal = {};
    
    public LED(){}

    public void initialize(){
        ledsignal[0] = 0x00;
    }

    public void execute(){
    int SERVICE_PORT = 8888;

      if(IntakeGamePiece.m_HopperState == HopperPiece.CONE){
        ledsignal[0] = 0x01;
      }
      else if(IntakeGamePiece.m_HopperState == HopperPiece.CUBE){
        ledsignal[0] = 0x02;
      }
      else{
        ledsignal[0] = 0x00;
      }

    try{
      
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
     }
}
