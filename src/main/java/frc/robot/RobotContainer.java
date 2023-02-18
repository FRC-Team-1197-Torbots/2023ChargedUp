// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DriveTrainConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ArcadeDrive;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveTrain DriveTrainSubsystem;
  private ArcadeDrive arcadeDrive;

  public static XboxController player1;
  public static XboxController player2;

  SendableChooser<Command> chooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    DriveTrainSubsystem = new DriveTrain();
    arcadeDrive = new ArcadeDrive(DriveTrainSubsystem);
    //IntakeSubsystem = new Intake();
    //moveIntake = new MoveIntake(IntakeSubsystem);

    player1 = new XboxController(0);
    player2 = new XboxController(1);

    DriveTrainSubsystem.setDefaultCommand(arcadeDrive);
    //IntakeSubsystem.setDefaultCommand(moveIntake);

    //initialize_Subsystems();
    configureButtonBindings();
    //configureBindings();
    chooser.addOption("Test Path: ", arcadeDrive);
    chooser.addOption("Test Path 2: ", arcadeDrive);

    Shuffleboard.getTab("Autonomous").add(chooser);
  }

  //public Command loadPathPlannerTrajectoryToRamseteCommand(String filename, boolean resetOdometry, Trajectory trajectory; );

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureButtonBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(DriveTrainSubsystem);
  }
}
