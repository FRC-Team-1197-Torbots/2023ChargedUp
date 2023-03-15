package frc.robot.commands.Drive;

import frc.robot.subsystems.DriveTrain;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.AutoBalanceConstants;
import frc.robot.Constants.TeleopDriveConstants;

public class AutoBalance extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final DriveTrain driveSubsystem;
    private DoubleSupplier m_throttle, m_steer;
    private double throttle, steer;
    private double previousThrottle;
  
    private double leftCurrentSpeed;
    private double rightCurrentSpeed;
  
  
    private double leftSpeed;
    private double rightspeed;
    private double leftTargetSpeed;
    private double rightTargetSpeed;
  
    private double leftOutput;
    private double rightOutput;
  
    private PIDController OutputCalculator;
  
    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public AutoBalance(DriveTrain subsystem) {
      driveSubsystem = subsystem;
      OutputCalculator = new PIDController(AutoBalanceConstants.BalancekP,AutoBalanceConstants.BalancekI,AutoBalanceConstants.BalancekD);
      // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(subsystem);
    }
    @Override
    public void execute(){
        leftOutput = OutputCalculator.calculate(0-driveSubsystem.getHeading());
        rightOutput = leftOutput;
        if (Math.abs(leftOutput) < 0.01 && Math.abs(rightOutput) < 0.01) {
            driveSubsystem.setMotorSpeeds(0, 0);
      }
      else {
            driveSubsystem.setMotorSpeeds(-leftOutput, -rightOutput);

    }
}
    @Override
    public void end(boolean interrupted){
        driveSubsystem.setMotorSpeeds(0, 0);
    }
}


