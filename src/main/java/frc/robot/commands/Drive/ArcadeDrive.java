// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import frc.robot.subsystems.DriveTrain;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.TeleopDriveConstants;

/** An example command that uses an example subsystem. */
public class ArcadeDrive extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveTrain driveSubsystem;
  private XboxController m_player1;
  private double throttle, steer;


  
  private DoubleSupplier m_throttle;
  private DoubleSupplier m_steer;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ArcadeDrive(DriveTrain subsystem, DoubleSupplier throttleInput, DoubleSupplier steerInput) {
    driveSubsystem = subsystem;
    //m_player1 = player1;
    m_throttle = throttleInput;
    m_steer = steerInput;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

// Called when the command is initially scheduled.
@Override
public void initialize() {}

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
  throttle = m_throttle.getAsDouble();
  steer = m_steer.getAsDouble();
  driveSubsystem.Drive(throttle, steer);
  
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
