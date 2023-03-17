// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.AutoDriveConstants;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.Constants.TeleopDriveConstants;
import frc.robot.Constants.ElevatorArmConstants.GamePiece;
import frc.robot.Constants.ElevatorArmConstants.STATE;
import frc.robot.Constants.ElevatorArmConstants.TARGET;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;

import java.io.IOException;
import java.nio.file.Path;
import java.util.HashMap;
import java.util.function.BiConsumer;
import java.util.function.Consumer;

import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.RamseteAutoBuilder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pneumatics;
import frc.robot.commands.Arm.RunArm;
import frc.robot.commands.Autos.TestAuto;
import frc.robot.commands.Drive.ArcadeDrive;
import frc.robot.commands.Elevator.RunElevator;
import frc.robot.commands.Elevator.SetElevatorState;
import frc.robot.commands.Intake.AutoIntakeCone;
import frc.robot.commands.Intake.AutoIntakeCube;
import frc.robot.commands.Intake.IntakeGamePiece;
import frc.robot.commands.Intake.SetIntakeMode;
import frc.robot.commands.Pneumatics.RunCompressor;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  public static CommandXboxController player1 = new CommandXboxController(0);
  private XboxController player1_HoldButton = new XboxController(0);
  private XboxController player2_HoldButton = new XboxController(1);
  public static CommandXboxController player2 = new CommandXboxController(1);

  HashMap<String, Command> eventMap = new HashMap<>();

  private RamseteAutoBuilder m_autoBuilder;
  private final SendableChooser<Command> m_autoChooser = new SendableChooser<>();
  private final DriveTrain driveSubsystem = new DriveTrain();
  private final Elevator elSubsystem = new Elevator();
  private final Arm armSystem = new Arm();
  private final Claw clawSystem = new Claw();
  private final Intake intakeSystem = new Intake();
  private final Pneumatics pneumaticsSystem = new Pneumatics();
  private RunCompressor runCompressor = new RunCompressor(pneumaticsSystem);
  //private ArcadeDrive arcadeDrive = new ArcadeDrive(driveSubsystem, () -> player1.getLeftY(), () -> player1.getLeftY());
  private RunArm runArm = new RunArm(armSystem, clawSystem, player1_HoldButton);
  private GamePiece m_gamePiece;
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    initAutoBuilder();
    initializeSubsystems();
    configureButtonBindings();
    System.out.println("RobotContainer Initialized");
  }

  public void initializeSubsystems(){
    System.out.println("Pneumatic System initialized");
    //driveSubsystem.setDefaultCommand(new ArcadeDrive(driveSubsystem, () -> player1.getLeftY(), () -> player1.getLeftX()));
    pneumaticsSystem.setDefaultCommand(runCompressor);
    //elSubsystem.setDefaultCommand(new RunElevator(armSystem, clawSystem, elSubsystem, ElevatorLevel.BOTTOM));
    //armSystem.setDefaultCommand(runArm);
  }

  private void initAutoBuilder() {
    eventMap.put("wait", new WaitCommand(5));
    //eventMap.put("IntakeCone", new AutoIntakeCone(intakeSystem, 0.5));
   // eventMap.put("IntakeCube", new AutoIntakeCube(intakeSystem, 0.25));
    Subsystem[] subArray = {driveSubsystem};

    m_autoBuilder =
        new RamseteAutoBuilder(
            driveSubsystem::getPose,
            driveSubsystem::resetOdometry,
            new RamseteController(AutoDriveConstants.kRamseteB, AutoDriveConstants.kRamseteZeta),
            new DifferentialDriveKinematics(DriveTrainConstants.kTrackWidthMeters), 
            driveSubsystem.getFeedForward(),
            driveSubsystem::getWheelSpeeds,
            new PIDConstants(TeleopDriveConstants.velocitykP, TeleopDriveConstants.velocitykI, TeleopDriveConstants.velocitykD),
            driveSubsystem::arcadeDriveVolts, 
            eventMap, 
            false, 
            subArray); 
            
              
}

public void initializeAutoChooser(){
  //m_autoChooser.addOption("TestAuto", new TestAuto(m_autoBuilder, DriveTrainSubsystem));
}
  

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
    
    player1.rightTrigger(0.1).whileTrue(new IntakeGamePiece(intakeSystem, GamePiece.CONE));
    //player1.y().whileTrue(new RunElevator(armSystem, clawSystem, elSubsystem, ElevatorLevel.MIDDLE));
    //player1.a().whileTrue(new RunElevator(armSystem, clawSystem, elSubsystem, ElevatorLevel.TOP));
    

    //player1.leftBumper(null)//(new RunArm(armSystem, clawSystem, 0.2));
    //player1.pov(0).whileTrue(new RunArm(armSystem, clawSystem, 0.35));//elSubsystem, 0.1));
    //player1.pov(180).onTrue(new RunArm(armSystem, clawSystem, -0.35));//elSubsystem, -0.1));
    player1.b().onTrue(new RunCompressor(pneumaticsSystem));
    player2.povUp().onTrue(new SetElevatorState(elSubsystem, TARGET.TOP));
    player2.povDown().onTrue(new SetElevatorState(elSubsystem, TARGET.MIDDLE));
    player2.y().onTrue(new SetIntakeMode(intakeSystem, GamePiece.CONE));
    player2.x().onTrue(new SetIntakeMode(intakeSystem, GamePiece.CUBE));
    //player1.a().onTrue(new AutoIntakeCone(intakeSystem, 0.2, true));

    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return m_autoChooser.getSelected();
  }
  public void testPeriodic(){
    if(player1_HoldButton.getAButton()){// A1 drive forward
      driveSubsystem.SetLeft(0.2);
      driveSubsystem.SetRight(0.2);
    }
    if(player1_HoldButton.getBButtonPressed()){//B1 drive backwards
      driveSubsystem.SetLeft(0.2);
      driveSubsystem.SetRight(0.2);
    }
    /* 
    if(player1_HoldButton.getXButtonPressed()){//X1 Arm up
      armSystem.SetArmSpeed(0.2);
    }
    if(player1_HoldButton.getYButtonPressed()){//Y1 Arm down
      armSystem.SetArmSpeed(-0.2);
    }  
    */  
    if(player2_HoldButton.getAButtonPressed()){//A2 Elevator Up
      elSubsystem.SetElevatorSpeed(0.2);
    }
    if(player2_HoldButton.getXButtonPressed()){//X2 Elevator Down
      elSubsystem.SetElevatorSpeed(-0.2);
    }



    SmartDashboard.putNumber("Drive Left Encoder", driveSubsystem.getLeftEncoder());
    SmartDashboard.putNumber("Drive Right Encoder", driveSubsystem.getRightEncoder());
    SmartDashboard.putNumber("Gyro Angle", driveSubsystem.getHeading());
    //SmartDashboard.putNumber("Encoder Value", elSubsystem.GetElevatorPos());
    //SmartDashboard.putNumber("Encoder Rate", elSubsystem.GetEncoderRate());
    //hi there
  }
}
