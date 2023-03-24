// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.REVPhysicsSim;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoDriveConstants;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.Constants.TeleopDriveConstants;
import frc.robot.commands.Drive.ArcadeDrive;

public class DriveTrain extends SubsystemBase {
  private ArcadeDrive arcadeDrive;

  private CANSparkMax LeftTop;
  private CANSparkMax LeftBottom1;
  private CANSparkMax LeftBottom2;

  private CANSparkMax RightTop;
  private CANSparkMax RightBottom1;
  private CANSparkMax RightBottom2;

  private double previousThrottle;

  private double leftCurrentSpeed;
  private double rightCurrentSpeed;


  private double leftSpeed;
  private double rightspeed;
  private double leftTargetSpeed;
  private double rightTargetSpeed;

  private double leftOutput;
  private double rightOutput;

  //public static Solenoid driveSolenoid;
  public static Compressor compressor;

  public static Encoder leftEncoder;
  public static Encoder rightEncoder;

  private final DifferentialDriveOdometry m_odometry;

  private AHRS gyro;
  private final DifferentialDrivePoseEstimator poseEstimator;
  private Pigeon2 pigeon;
  private final Field2d m_field = new Field2d();

  private PIDController pidDrive;

  
  private final SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(AutoDriveConstants.ksVolts, 
  AutoDriveConstants.kVoltSecondPerMeter, 
  AutoDriveConstants.kVoltSecondSquaredPerMeter);


  //Simulation Objects
  private EncoderSim leftEncoderSim;
  private EncoderSim rightEncoderSim;
  static final double KvLinear = 1.98;
  static final double KaLinear = 0.2;
  static final double KvAngular = 1.5;
  static final double KaAngular = 0.3;
  //private AnalogGyro m_gyro;
  //private AnalogGyroSim gyroSim;
  private final LinearSystem<N2, N2, N2> m_drivetrainSystem;
  private final DifferentialDrivetrainSim m_drivetrainSimulator;
  //private SimDevice navx_sim;
  int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
  SimDouble angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));

  /** Creates a new ExampleSubsystem. */
  public DriveTrain() {
    //navx_sim = new SimDevice(dev);
    
    pigeon = new Pigeon2(0);
    gyro = new AHRS(SPI.Port.kMXP);

    pidDrive = new PIDController(TeleopDriveConstants.velocitykP, TeleopDriveConstants.velocitykI, TeleopDriveConstants.velocitykD);
    //m_gyro = new AnalogGyro(0);

    LeftTop = new CANSparkMax(DriveTrainConstants.LeftTopID, MotorType.kBrushless);
		LeftBottom1 = new CANSparkMax(DriveTrainConstants.LeftBottom1ID, MotorType.kBrushless);
		LeftBottom2 = new CANSparkMax(DriveTrainConstants.LeftBottom2ID, MotorType.kBrushless); 

		RightTop = new CANSparkMax(DriveTrainConstants.RightTopID, MotorType.kBrushless); 
		RightBottom1 = new CANSparkMax(DriveTrainConstants.RightBottom1ID, MotorType.kBrushless);		
		RightBottom2 = new CANSparkMax(DriveTrainConstants.RightBottom2ID, MotorType.kBrushless);

    //driveSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 0);

    leftEncoder = new Encoder(6, 7, false, Encoder.EncodingType.k4X);
		rightEncoder = new Encoder(0, 1, false, Encoder.EncodingType.k4X);

    //rightEncoder.
    m_odometry = new DifferentialDriveOdometry(gyro.getRotation2d(), getLeftEncoder(), getRightEncoder());
    poseEstimator = new DifferentialDrivePoseEstimator(new DifferentialDriveKinematics(0), new Rotation2d(), getHeading(), getAverageEncoder(), getPose());
    SmartDashboard.putData("Field 2D", m_field);
    
    resetEncoder();
    resetGyro();

    m_odometry.resetPosition(gyro.getRotation2d(), getLeftEncoder(), getRightEncoder(), new Pose2d());

    //Simulation Initializing
    m_drivetrainSystem = LinearSystemId.identifyDrivetrainSystem(1.98, 0.2, 1.5, 0.3);
    m_drivetrainSimulator =
    new DifferentialDrivetrainSim(
        m_drivetrainSystem, DCMotor.getCIM(2), 8, DriveTrainConstants.kTrackWidthMeters, DriveTrainConstants.kWheelRadiusInches, null);
    leftEncoderSim = new EncoderSim(leftEncoder);
    rightEncoderSim = new EncoderSim(rightEncoder);
    //gyroSim = new AnalogGyroSim(m_gyro);
    
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */

  public void Drive(double throttle, double steer){
    double sign = Math.signum(throttle);
  throttle = sign * Math.pow(throttle, 2);
  //System.out.println("Throttle: " + throttle);

 
 sign = Math.signum(steer);
  steer = sign * Math.pow(steer, 2);// * TeleopDriveConstants.STEER_SCALAR;  
  //System.out.println("Steer: " + steer);

     if(Math.abs(throttle) < 0.025f) {
          throttle = 0;
     }

     if(Math.abs(steer) < 0.035f) {
         steer = 0;
     }

     double rightspeed = 0, leftSpeed = 0;

     /* 
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
      }*/

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

    leftTargetSpeed = settingLeftSpeed; //* TeleopDriveConstants.MAX_VELOCITY;
    rightTargetSpeed = settingRightSpeed; //* TeleopDriveConstants.MAX_VELOCITY;

    leftOutput = 0.5; //= pidDrive.calculate(leftTargetSpeed - leftCurrentSpeed);
    rightOutput = 0.5;//= pidDrive.calculate(rightTargetSpeed - rightCurrentSpeed);

    System.out.println("Left output: " + leftOutput);
    if (Math.abs(leftOutput) < 0.01 && Math.abs(rightOutput) < 0.01) {
          setMotorSpeeds(0, 0);
    }
    else {
          setMotorSpeeds(-leftOutput, -rightOutput);
    }

    
  }

  public void setMotorState(IdleMode mode){
    LeftTop.setIdleMode(mode);
    LeftBottom1.setIdleMode(mode);
    LeftBottom2.setIdleMode(mode);
    RightTop.setIdleMode(mode);
    RightBottom1.setIdleMode(mode);
    RightBottom2.setIdleMode(mode);
  }

  /* 
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
*/
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
    return m_odometry.getPoseMeters();
  }


  public void setOdometry(Pose2d pose) {
    gyro.setAngleAdjustment(pose.getRotation().getDegrees());
    m_odometry.resetPosition(gyro.getRotation2d(), getLeftEncoder(), getRightEncoder(), pose);
  }

  public void resetOdometry(Pose2d pose){
    resetEncoder();
    m_odometry.resetPosition(gyro.getRotation2d(), getLeftEncoder(), getRightEncoder(), pose);
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds(){
    return new DifferentialDriveWheelSpeeds(getLeftVelocity(), getRightVelocity());
  }

  public SimpleMotorFeedforward getFeedForward(){
    return feedForward;
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
    //System.out.println("Signal");
    
    //m_odometry.update(gyro.getRotation2d(), getLeftEncoder(), getRightEncoder());

    SmartDashboard.putNumber("Left Encoder Value Meters: ", getLeftEncoder());
    SmartDashboard.putNumber("Right Encoder Value Meters", getRightEncoder());
    //SmartDashboard.putNumber("Gyro Heading", getHeading());
    //SmartDashboard.putString("Robot Pose", m_field.getRobotPose().toString());

    m_field.setRobotPose(m_odometry.getPoseMeters());
    m_field.setRobotPose(getPose());
    //m_field.getObject("traj").setTrajectory(PathPlanner.loadPath("BlueBottomIntakeCone", 2, 2));

  }

  public void simulationInit(){
    REVPhysicsSim.getInstance().addSparkMax(LeftTop, DCMotor.getNEO(1));
    REVPhysicsSim.getInstance().addSparkMax(RightTop, DCMotor.getNEO(1));
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    REVPhysicsSim.getInstance().run();
    m_drivetrainSimulator.setInputs(LeftTop.get() * RobotController.getInputCurrent(),
    RightTop.get() * RobotController.getInputCurrent());
    m_drivetrainSimulator.update(.02);
    leftEncoderSim.setDistance(m_drivetrainSimulator.getLeftPositionMeters());
    leftEncoderSim.setRate(m_drivetrainSimulator.getLeftVelocityMetersPerSecond());
    rightEncoderSim.setDistance(m_drivetrainSimulator.getRightPositionMeters());
    rightEncoderSim.setRate(m_drivetrainSimulator.getRightVelocityMetersPerSecond());
    //SmartDashboard.putNumber("Left Encoder Sim: ", leftEncoderSim.getDistance());
    //SmartDashboard.putNumber("Right Encoder Sim: ", rightEncoderSim.getDistance());
    //gyroSim.setAngle(driveSim.getHeading().getDegrees());
    //System.out.println(dev);
    //System.out.println("Angle" + angle.get());
    angle.set(m_drivetrainSimulator.getHeading().getDegrees());
    System.out.println(getPose());
    //angle.set(50);
    //m_field.setRobotPose(getPose());
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
		LeftTop.set(speed);
		LeftBottom1.set(speed);
		LeftBottom2.set(speed);
	}

	// Setting the right master Talon's speed to the given parameter
	public void SetRight(double speed) {
		RightTop.set(-speed); //in correct setting, but "software fix"		
		RightBottom1.set(-speed);
		RightBottom2.set(-speed);
	}

  public void setMotorSpeeds(double leftSpeed, double rightSpeed){
    SetLeft(leftSpeed);
    SetRight(rightSpeed);
  }
/* 
  public void shiftToLowGear() {
		driveSolenoid.set(true);
	}
	
	// Method to shift the drive to high gear
	public void shiftToHighGear() {
		driveSolenoid.set(false);
	}
*/
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
