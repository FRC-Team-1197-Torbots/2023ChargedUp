package frc.robot.commands.Arm;

import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.ElevatorArmConstants;
//import frc.robot.Constants.ElevatorArmConstants.MoveElevatorArm;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;

public class RunArm extends CommandBase{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

    private Arm arm;
    private Claw claw;
    private Elevator elevator;
    private PIDController elevatorPID;
    private double m_speed;
    private XboxController m_player2;

    private double currentPosition;
    private Boolean StateAchieved = false;
    //private MoveElevatorArm moveArm;
    public static enum MoveArm{
        UP,DOWN
    }
    private MoveArm ArmState;

    public RunArm(Arm armSystem){//, XboxController player2, Elevator elevatorSystem, double speed){
        arm = armSystem;
        //claw = clawSystem;
        //m_player2 = player2;
        ArmState= MoveArm.DOWN;
        //elevator = elevatorSystem;
        //elevatorPID = new PIDController(0, 0, 0);
        addRequirements(armSystem);//, //elevatorSystem);
        //System.out.println("arm instantiate");
    }
    @Override
    public void initialize() {
        SmartDashboard.putNumber("Potentiometer Value", arm.GetPotValue());
        //elevator.ResetEncoder();
    }
    @Override
    public void execute(){
        
        //System.out.println("execute");
        SmartDashboard.putNumber("Potentiometer Value", arm.GetPotValue());
        SmartDashboard.putString("Arm State", ArmState.toString());
        SmartDashboard.putBoolean("Arm State achieved?", StateAchieved);
        /*if(RobotContainer.player2_HoldButton.getXButton()){
            ArmState = MoveArm.DOWN;
            //System.out.println("arm down");
        }
        else if(RobotContainer.player2_HoldButton.getBButton()){
            ArmState = MoveArm.UP;
            //System.out.println("arm up");
        }else{
            ArmState = MoveArm.IDLE;
            //System.out.println("arm idle");
        }*/
        switch(ArmState){
            case DOWN:
            while(arm.GetPotValue()<=0.98){
                if(arm.GetPotValue()>=0.9){
                    arm.SetArmSpeed(0.2);
                    StateAchieved=false;
                }
                else if(arm.GetPotValue()>=0.97){
                    arm.SetArmSpeed(0.01);
                    StateAchieved=true;
                }
                else{
                    arm.SetArmSpeed(0.4);}
                    StateAchieved=false;
                }
                
        
            /*
            if(arm.GetPotValue()>=0.98){
                arm.SetArmSpeed(0.015);
            }
            else{*
            arm.SetArmSpeed(0.4);
            }*/
               
                //System.out.println("switch down");
                break;
            case UP:
            while(arm.GetPotValue()>=0.6285){
                if(arm.GetPotValue()<=0.7){
                    arm.SetArmSpeed(-0.2);
                    StateAchieved=false;
                }
                else if(arm.GetPotValue()<=0.635){
                    arm.SetArmSpeed(-0.015);
                    StateAchieved=true;
                }
                else{
                    arm.SetArmSpeed(-0.3);}
                    StateAchieved=false;
                }
            /*
                if(arm.GetPotValue()<=0.6285){
                    arm.SetArmSpeed(-0.01);
                }
                else{
                arm.SetArmSpeed(-0.3);
                }
                //System.out.println("Switch up");*/
                break;
            /*case IDLE:
                arm.SetArmSpeed(-0.01);
                //System.out.println("Switch Idle");
                break;*/
        }
        //System.out.println("Encoder value " + elevator.GetEncoderValue());
        //System.out.println("Encoder rate " + elevator.GetEncoderRate());

        /*switch(moveArm){
            case IDLE:
            break;
            case ACTIVE:
            break;
        }
        if(m_player1.getYButton()){
            moveArm = MoveElevatorArm.ACTIVE;
        }
        else{
            moveArm = MoveElevatorArm.IDLE;
        }
        //x and b
        double speed;
        if(m_player1.getXButton()){
            double speed = 0.1;
        }
    }
        
    public void setSpeed(double speed){
        m_speed = speed;
        */
    }

    @Override
    public void end(boolean interrupted) {


    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;

    }
}//-9242 is around the max encoder value when elevator is fully extended

    

