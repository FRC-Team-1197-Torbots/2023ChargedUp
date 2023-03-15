package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import com.revrobotics.CANSparkMax.IdleMode;

public class SetMotorMode extends CommandBase{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final DriveTrain m_DriveTrain;
    private final IdleMode m_mode;

  /**
   * Sets the drivetrain neutral mode (coast/brake).
   *
   * @param driveTrain The driveTrain used by this command.
   * @param mode {@link DriveTrainNeutralMode}: COAST, BRAKE, or HALF_BRAKE.
   */
    public SetMotorMode(DriveTrain driveTrain, IdleMode mode) {
        m_DriveTrain = driveTrain;
        m_mode = mode;
    // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(driveTrain);
  }

  @Override
  public void initialize() {
    m_DriveTrain.setMotorState(m_mode);;
  }
    
}
