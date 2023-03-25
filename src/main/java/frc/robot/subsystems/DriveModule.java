package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoDriveConstants;

public class DriveModule extends SubsystemBase{
    int module_num;
    CANSparkMax TopMotor;
    CANSparkMax BottomMotor1;
    CANSparkMax BottomMotor2;
    private final Encoder DriveEncoder;

    private double m_simDriveEncoderPosition;
    private double m_simDriveEncoderVelocity;
    private double m_simAngleDifference;
    private double m_simTurnAngleIncrement;
    Pose2d m_pose;

    SimpleMotorFeedforward feedforward =
          new SimpleMotorFeedforward(
                  AutoDriveConstants.ksVolts,
                  AutoDriveConstants.kVoltSecondPerMeter,
                  AutoDriveConstants.kVoltSecondSquaredPerMeter);

    public DriveModule(
        int moduleNumber,
        CANSparkMax topMotor,
        CANSparkMax bottomMotor1,
        CANSparkMax bottomMotor2,
        Encoder encoder
    ){
        module_num = moduleNumber;
        TopMotor = topMotor;
        BottomMotor1 = bottomMotor1;
        BottomMotor2 = bottomMotor2;
        DriveEncoder = encoder;
        if (RobotBase.isSimulation()) {
            REVPhysicsSim.getInstance().addSparkMax(TopMotor, DCMotor.getNEO(1));
            REVPhysicsSim.getInstance().addSparkMax(BottomMotor1, DCMotor.getNEO(1));
            REVPhysicsSim.getInstance().addSparkMax(BottomMotor2, DCMotor.getNEO(1));
            //m_driveController.setP(1, SIM_SLOT);
          }

    }
    
}
