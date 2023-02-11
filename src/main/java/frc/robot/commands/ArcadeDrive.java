// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
/** An example command that uses an example subsystem. */
public class ArcadeDrive extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveTrain driveSubsystem;
  private double throttle;
  private double steer;

  private double leftSpeed;
  private double rightspeed;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ArcadeDrive(DriveTrain subsystem) {
    driveSubsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveSubsystem.compressor.enableDigital();
  if(RobotContainer.player1.getAButtonPressed() && driveSubsystem.solenoid.get()){
      driveSubsystem.shiftToLowGear();
  }
  if(RobotContainer.player1.getYButtonPressed() && !driveSubsystem.solenoid.get()){
      driveSubsystem.shiftToHighGear();
  }
  throttle = -RobotContainer.player1.getRawAxis(1);
  steer = RobotContainer.player1.getRawAxis(0);
  System.out.println("Throttle: " + throttle);
  System.out.println("Steer: " + steer);
  
  if(Math.abs(throttle) < 0.03f) {
      throttle = 0;
 }

 if(Math.abs(steer) < 0.03f) {
     steer = 0;
 }

 if(throttle > 1) {
  throttle = 1;
  }

  if(throttle < -1) {
      throttle = -1;
  }
  

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
  //System.out.printf("%-10s %f %10s %f", "Left Speed", leftSpeed, "Right Speed", rightspeed);

  driveSubsystem.setMotorSpeeds(leftSpeed, rightspeed);
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
