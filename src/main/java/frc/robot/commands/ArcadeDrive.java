// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.TeleopDriveConstants;

/** An example command that uses an example subsystem. */
public class ArcadeDrive extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveTrain driveSubsystem;
  private double throttle;
  private double steer;
  private double previousThrottle;

  private double leftCurrentSpeed;
  private double rightCurrentSpeed;


  private double leftSpeed;
  private double rightspeed;
  private double leftTargetSpeed;
  private double rightTargetSpeed;

  private double leftOutput;
  private double rightOutput;

  private PIDController pidDrive;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ArcadeDrive(DriveTrain subsystem) {
    driveSubsystem = subsystem;
    pidDrive = new PIDController(TeleopDriveConstants.velocitykP, TeleopDriveConstants.velocitykI, TeleopDriveConstants.velocitykD);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

// Called when the command is initially scheduled.
@Override
public void initialize() {}

// Called every time the scheduler runs while the command is scheduled.
@Override
public void execute() {
    driveSubsystem.compressor.enableDigital();
    if(RobotContainer.player1.getAButtonPressed() && driveSubsystem.driveSolenoid.get()){
        driveSubsystem.shiftToLowGear();
    }
    if(RobotContainer.player1.getYButtonPressed() && !driveSubsystem.driveSolenoid.get()){
        driveSubsystem.shiftToHighGear();
    }
  throttle = -RobotContainer.player1.getRawAxis(1);
  double sign = Math.signum(throttle);
  throttle = sign * Math.pow(throttle, 2);

 steer = -RobotContainer.player1.getRawAxis(0);
 sign = Math.signum(steer);
  steer = sign * Math.pow(steer, 2) * TeleopDriveConstants.STEER_SCALAR;  

     if(Math.abs(throttle) < 0.025f) {
          throttle = 0;
     }

     if(Math.abs(steer) < 0.035f) {
         steer = 0;
     }

     double rightspeed = 0, leftSpeed = 0;

     if (throttle > previousThrottle) {
         if (previousThrottle> 0)
              throttle = previousThrottle + TeleopDriveConstants.POSRANGE_MAX_ACCEL;
          else{                
              throttle = previousThrottle + TeleopDriveConstants.NEGRANGE_MAX_ACCEL;
              //System.out.println("Throttle " + throttle);
          }
              
      }

      if (throttle < previousThrottle && Math.abs(throttle - previousThrottle) > TeleopDriveConstants.MAX_DECEL) {
          throttle = previousThrottle - TeleopDriveConstants.MAX_DECEL;
      }

      if(throttle > 1) {
          throttle = 1;
      }

      if(throttle < -1) {
          throttle = -1;
      }


      previousThrottle = throttle;
      
      throttle = -throttle;

     if(throttle > 0) {
         if(steer > 0) {
             leftSpeed = throttle - steer;
             rightspeed = Math.max(throttle, steer);
         } else {
             leftSpeed = Math.max(throttle, -steer);
             rightspeed = throttle + steer;
         }
     } else {
         if(steer > 0) {
             leftSpeed = -Math.max(-throttle, steer);
             rightspeed = throttle + steer;
         } else {
             leftSpeed = throttle - steer;
             rightspeed = -Math.max(-throttle, -steer);
         }
     }

      double settingLeftSpeed = leftSpeed, settingRightSpeed = rightspeed;

      if(Math.abs(settingLeftSpeed) < 0.05) {
          settingLeftSpeed = 0;
      }

      if(Math.abs(settingRightSpeed) < 0.05) {
          settingRightSpeed = 0;
      }

    leftCurrentSpeed = getLeftVelocity();
    rightCurrentSpeed = getRightVelocity();

    //System.out.println("Left Encoder: " + getLeftEncoder());
    //System.out.println("Right Encoder: " + getRightEncoder());

    leftTargetSpeed = settingLeftSpeed * TeleopDriveConstants.MAX_VELOCITY;
    rightTargetSpeed = settingRightSpeed * TeleopDriveConstants.MAX_VELOCITY;

    leftOutput = pidDrive.calculate(leftTargetSpeed - leftCurrentSpeed);
    rightOutput = pidDrive.calculate(rightTargetSpeed - rightCurrentSpeed);


    if (Math.abs(leftOutput) < 0.01 && Math.abs(rightOutput) < 0.01) {
          driveSubsystem.setMotorSpeeds(0, 0);
    }
    else {
          driveSubsystem.setMotorSpeeds(-leftOutput, -rightOutput);
    }
  }

  public double getRightVelocity() {
    return driveSubsystem.rightEncoder.getRate();
  }
  
  public double getLeftVelocity() {
    return -driveSubsystem.leftEncoder.getRate();
  }
  
  public double getLeftEncoder() {
    return -driveSubsystem.leftEncoder.getRaw();
  }
  
  public double getRightEncoder() {
    return driveSubsystem.rightEncoder.getRaw();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;

  }
}
