// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.DriveTrainConstants;
import frc.robot.commands.ArcadeDrive;

public class DriveTrain extends SubsystemBase {
  private ArcadeDrive arcadeDrive;

  private CANSparkMax LeftTop;
  private CANSparkMax LeftBottom1;
  private CANSparkMax LeftBottom2;

  private CANSparkMax RightTop;
  private CANSparkMax RightBottom1;
  private CANSparkMax RightBottom2;

  public static Solenoid solenoid;
  public static Compressor compressor;

  public static Encoder leftEncoder;
  public static Encoder rightEncoder;

  private final DifferentialDriveOdometry m_Odometry;

  private AHRS gyro;

  /** Creates a new ExampleSubsystem. */
  public DriveTrain() {
    gyro = new AHRS(SPI.Port.kMXP);

    LeftTop = new CANSparkMax(DriveTrainConstants.LeftTopID, MotorType.kBrushless);
		LeftBottom1 = new CANSparkMax(DriveTrainConstants.LeftBottom1ID, MotorType.kBrushless);
		LeftBottom2 = new CANSparkMax(DriveTrainConstants.LeftBottom2ID, MotorType.kBrushless); 

		RightTop = new CANSparkMax(DriveTrainConstants.RightTopID, MotorType.kBrushless); 
		RightBottom1 = new CANSparkMax(DriveTrainConstants.RightBottom1ID, MotorType.kBrushless);		
		RightBottom2 = new CANSparkMax(DriveTrainConstants.RightBottom2ID, MotorType.kBrushless);

    solenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 0);
    compressor = new Compressor(PneumaticsModuleType.CTREPCM);

    leftEncoder = new Encoder(0, 1, false, Encoder.EncodingType.k4X);
		rightEncoder = new Encoder(2, 3, false, Encoder.EncodingType.k4X);

    //rightEncoder.

    resetEncoder();
    resetGyro();

    m_Odometry = new DifferentialDriveOdometry(gyro.getRotation2d(), getLeftEncoder(), getRightEncoder());
    m_Odometry.resetPosition(gyro.getRotation2d(), getLeftEncoder(), getRightEncoder(), new Pose2d());
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */

  public void setBreakMode(){
    LeftTop.setIdleMode(IdleMode.kBrake);
    LeftBottom1.setIdleMode(IdleMode.kBrake);
    LeftBottom2.setIdleMode(IdleMode.kBrake);
    RightTop.setIdleMode(IdleMode.kBrake);
    RightBottom1.setIdleMode(IdleMode.kBrake);
    RightBottom2.setIdleMode(IdleMode.kBrake);
  }

  public void setCoastMode(){
    LeftTop.setIdleMode(IdleMode.kCoast);
    LeftBottom1.setIdleMode(IdleMode.kCoast);
    LeftBottom2.setIdleMode(IdleMode.kCoast);
    RightTop.setIdleMode(IdleMode.kCoast);
    RightBottom1.setIdleMode(IdleMode.kCoast);
    RightBottom2.setIdleMode(IdleMode.kCoast);
  }

  public double getRightVelocity() {
    return rightEncoder.getRate();
  }
  
  public double getLeftVelocity() {
    return -leftEncoder.getRate();
  }
  
  public double getLeftEncoder() {
    return -leftEncoder.getRaw();
  }
  
  public double getRightEncoder() {
    return rightEncoder.getRaw();
  }

  public double getAverageEncoder(){
    return ((getLeftEncoder() + getRightEncoder()) / 2.0);
  }

  public double getHeading(){
    return gyro.getRotation2d().getDegrees();
  }

  public double getTurnRate(){
    return -gyro.getRate();
  }

  public Pose2d getPose(){
    return m_Odometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose){
    resetEncoder();
    m_Odometry.resetPosition(gyro.getRotation2d(), getLeftEncoder(), getRightEncoder(), pose);
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds(){
    return new DifferentialDriveWheelSpeeds(getLeftVelocity(), getRightVelocity());
  }

  public void arcadeDriveVolts(double leftVolts, double rightVolts){
    setLeftVoltage(leftVolts);
    setRightVoltage(rightVolts);
  }

  public void setLeftVoltage(double leftVolts){
    LeftTop.setVoltage(leftVolts);
    LeftBottom1.setVoltage(leftVolts);
    LeftBottom2.setVoltage(leftVolts);
  }

  public void setRightVoltage(double rightVolts){
    LeftTop.setVoltage(rightVolts);
    LeftBottom1.setVoltage(rightVolts);
    LeftBottom2.setVoltage(rightVolts);
  }

  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_Odometry.update(gyro.getRotation2d(), getLeftEncoder(), getRightEncoder());

    SmartDashboard.putNumber("Left Encoder Value Meters: ", getLeftEncoder());
    SmartDashboard.putNumber("Right Encoder Value Meters", getRightEncoder());
    SmartDashboard.putNumber("Gyro Heading", getHeading());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
  
  // Method to reset the encoder values
	public void resetEncoder() {
		leftEncoder.reset();
		rightEncoder.reset();
	}

	// Method to reset the spartan board gyro values
	public void resetGyro() {
		gyro.reset();
    gyro.calibrate(); 
	}

  public void SetLeft(double speed) {
		LeftTop.set(-speed);
		LeftBottom1.set(speed);
		LeftBottom2.set(speed);
	}

	// Setting the right master Talon's speed to the given parameter
	public void SetRight(double speed) {
		RightTop.set(speed); //in correct setting, but "software fix"		
		RightBottom1.set(-speed);
		RightBottom2.set(-speed);
	}

  public void setMotorSpeeds(double leftSpeed, double rightSpeed){
    SetLeft(leftSpeed);
    SetRight(rightSpeed);
  }

  public void shiftToLowGear() {
		solenoid.set(true);
	}
	
	// Method to shift the drive to high gear
	public void shiftToHighGear() {
		solenoid.set(false);
	}

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public CommandBase exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }
}
