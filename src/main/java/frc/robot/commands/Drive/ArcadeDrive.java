// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import frc.robot.subsystems.DriveTrain;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.TeleopDriveConstants;

/** An example command that uses an example subsystem. */
public class ArcadeDrive extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveTrain driveSubsystem;
  private XboxController m_player1;
  private double throttle, steer;
  private double maxThrottle, maxSteer;

  
  private DoubleSupplier m_throttle;
  private DoubleSupplier m_steer;
  private boolean low;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ArcadeDrive(DriveTrain subsystem, DoubleSupplier throttleInput, DoubleSupplier steerInput, XboxController player1) {
    driveSubsystem = subsystem;
    //m_player1 = player1;
    m_throttle = throttleInput;
    m_steer = steerInput;
    m_player1 = player1;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);

    low = false;
  }

// Called when the command is initially scheduled.
@Override
public void initialize() {
  driveSubsystem.setMotorState(IdleMode.kBrake);
}

// Called every time the scheduler runs while the command is scheduled.
@Override
public void execute() {
  
  /* 
    driveSubsystem.compressor.enableDigital();
    if(RobotContainer.player1.getXButtonPressed() && driveSubsystem.driveSolenoid.get()){
        driveSubsystem.shiftToLowGear();
    }
    if(RobotContainer.player1.getBButtonPressed() && !driveSubsystem.driveSolenoid.get()){
        driveSubsystem.shiftToHighGear();
    }
    */

    if(m_player1.getRawButtonReleased(1)) {
      low = !low;
    }

    SmartDashboard.putBoolean("low", low);

  if(!low){
    maxThrottle = 0.9;
    maxSteer = 0.9;
  } else if(low){
    maxThrottle = 0.5;
    maxSteer = 0.5;
  }
  
  throttle = m_throttle.getAsDouble();
  steer = m_steer.getAsDouble();
  double sign = Math.signum(throttle);
  throttle = sign * Math.pow(throttle, 2) * maxThrottle;
  sign = Math.signum(steer);
  steer = sign * Math.pow(steer, 2) * maxSteer;

  //steer *= -1;
  
  if(Math.abs(throttle) < 0.025f) {
    throttle = 0;
  }

  if(Math.abs(steer) < 0.035f) {
   steer = 0;
  }

  double leftSpeed, rightspeed;

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

// System.out.println("Left speed: " + leftSpeed);
// System.out.println("Right Speed: " + rightspeed);
// System.out.println("Left Encoder rate: " + driveSubsystem.getLeftVelocity());
// System.out.println("Right Encoder rate: " + driveSubsystem.getRightVelocity());

  //driveSubsystem.SetLeft(leftSpeed);
  //driveSubsystem.SetRight(rightspeed);
  driveSubsystem.Drive(leftSpeed, rightspeed);
  
  
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
