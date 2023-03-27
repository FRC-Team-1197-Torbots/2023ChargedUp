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
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
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
  private double m_throttle;
  private double m_steer;

  private double leftCurrentSpeed;
  private double rightCurrentSpeed;


  private double leftSpeed;
  private double rightspeed;
  private double leftTargetSpeed;
  private double rightTargetSpeed;
  private double lefterror;
  private double righterror;

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

  private DifferentialDriveKinematics m_DriveKinematics = new DifferentialDriveKinematics(DriveTrainConstants.kTrackWidth);
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

  private double m_simYaw;
  //private AnalogGyro m_gyro;
  //private AnalogGyroSim gyroSim;
  private final LinearSystem<N2, N2, N2> m_drivetrainSystem;
  private final DifferentialDrivetrainSim m_drivetrainSimulator;
  private final PIDController m_leftPIDController = new PIDController((1.0/17000.0), 0, 0.000000000);
  private final PIDController m_rightPIDController = new PIDController((1.0/17000.0), 0, 0.000000000);
  private final MotorControllerGroup leftMotorControllerGroup;
  private final MotorControllerGroup rightMotorControllerGroup;
  private double lefttemp;
  private double righttemp;
  //private MotorController[] leftMotorArray = {LeftTop, LeftBottom1, LeftBottom2};
  //private MotorController[] rightMotorArray = {RightTop, RightBottom1, RightBottom2};
  //private DifferentialDrive m_Drive; 

  //private SimDevice navx_sim;
  int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
  SimDouble angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));

  /** Creates a new ExampleSubsystem. */
  public DriveTrain() {
    //navx_sim = new SimDevice(dev);
    LeftTop = new CANSparkMax(DriveTrainConstants.LeftTopID, MotorType.kBrushless);
		LeftBottom1 = new CANSparkMax(DriveTrainConstants.LeftBottom1ID, MotorType.kBrushless);
		LeftBottom2 = new CANSparkMax(DriveTrainConstants.LeftBottom2ID, MotorType.kBrushless); 

		RightTop = new CANSparkMax(DriveTrainConstants.RightTopID, MotorType.kBrushless); 
		RightBottom1 = new CANSparkMax(DriveTrainConstants.RightBottom1ID, MotorType.kBrushless);		
		RightBottom2 = new CANSparkMax(DriveTrainConstants.RightBottom2ID, MotorType.kBrushless);

    leftMotorControllerGroup = new MotorControllerGroup(LeftTop, LeftBottom1, LeftBottom2);
    rightMotorControllerGroup = new MotorControllerGroup(RightTop, RightBottom1, RightBottom2);
    //m_Drive = new DifferentialDrive(LeftBottom1, RightTop);
    
    pigeon = new Pigeon2(1);
    gyro = new AHRS(SPI.Port.kMXP);

    pidDrive = new PIDController(TeleopDriveConstants.velocitykP, TeleopDriveConstants.velocitykI, TeleopDriveConstants.velocitykD);
    //m_gyro = new AnalogGyro(0);

    //driveSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 0);

    leftEncoder = new Encoder(8, 9, false, Encoder.EncodingType.k4X);
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
        m_drivetrainSystem, DCMotor.getNeo550(6), 8, DriveTrainConstants.kTrackWidth, DriveTrainConstants.kWheelRadiusInches, null);
    leftEncoderSim = new EncoderSim(leftEncoder);
    rightEncoderSim = new EncoderSim(rightEncoder);
    //pigeon.zeroGyroBiasNow()
    pigeon.configFactoryDefault();
    pigeon.setYaw(0);
    //gyroSim = new AnalogGyroSim(m_gyro);
    
  }

  public void setpigeon(){
    // System.out.println("Pigeon Pitch: " + pigeon.getPitch() + 
    // "Pigeon Yaw: " + pigeon.getYaw() + 
    // "Pigeon Roll: " + pigeon.getRoll());
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */

  public void Drive(double throttle, double steer){
    double leftsign;
    double rightsign;
    

    leftCurrentSpeed = getLeftVelocity();
    //System.out.println("Left Position: " + leftEncoder.getRaw());
    //System.out.println("Right Position: " + rightEncoder.getRaw());
    rightCurrentSpeed = getRightVelocity();
    SmartDashboard.putNumber("Left Velocity: ", leftCurrentSpeed);
    SmartDashboard.putNumber("Right Velocity: ", rightCurrentSpeed);
    //System.out.println("Left Speed: " + leftCurrentSpeed);
    //System.out.println("Right Speed: " + rightCurrentSpeed);
    leftTargetSpeed = throttle * TeleopDriveConstants.MAX_VELOCITY;
    rightTargetSpeed = steer * TeleopDriveConstants.MAX_VELOCITY;
    lefterror = leftTargetSpeed - leftCurrentSpeed;
    righterror = rightTargetSpeed - rightCurrentSpeed;
    SmartDashboard.putNumber("Left Error", lefterror);
    SmartDashboard.putNumber("Right Error", righterror);
    SmartDashboard.putNumber("Left Target", leftTargetSpeed);
    SmartDashboard.putNumber("Right Target", rightTargetSpeed);

    // if(Math.abs(lefterror) > 75000f && Math.abs(leftCurrentSpeed) < 75000f){
    //   leftsign = Math.signum(lefterror);
    //   lefterror = leftsign * 75000;
    // }
    // if(Math.abs(righterror) > 75000f && Math.abs(rightCurrentSpeed) < 75000f){
    //   rightsign = Math.signum(righterror);
    //   righterror = rightsign * 75000;
    // }
    //System.out.println("Error: " + lefterror + " " + righterror);
    //leftOutput = m_leftPIDController.calculate(lefterror);
    //rightOutput = m_rightPIDController.calculate(righterror);
    //System.out.println("Throttle: " + throttle);

    righttemp = 0.0000555 * righterror;//(1.0f/17000.0f) * righterror;
    lefttemp = 0.0000555 * lefterror;//(1.0f/17000.0f) * lefterror;

      SmartDashboard.putNumber("My PID Value", righttemp);

    //System.out.println("Left Output: " + leftOutput);
    //System.out.println("Right Output: " + rightOutput);
    //setMotorSpeeds(leftOutput, rightOutput);
    setMotorSpeeds(throttle, steer);
    /* 
    m_throttle = throttle;
    m_steer = steer;
    double sign = Math.signum(throttle);
    m_throttle = sign * Math.pow(throttle, 2);
  //System.out.println("Throttle: " + throttle);

 
  sign = Math.signum(steer);
  m_steer = sign * Math.pow(steer, 2);// * TeleopDriveConstants.STEER_SCALAR;  
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
      }

      if(throttle > 1) {
          throttle = 1;
      }

      if(throttle < -1) {
          throttle = -1;
      }


      //previousThrottle = throttle;
      
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

    leftOutput = leftSpeed; //= pidDrive.calculate(leftTargetSpeed - leftCurrentSpeed);
    rightOutput = rightspeed;//= pidDrive.calculate(rightTargetSpeed - rightCurrentSpeed);

    //System.out.println("Left output: " + rightOutput);
    if (Math.abs(leftOutput) < 0.01 && Math.abs(rightOutput) < 0.01) {
          setMotorSpeeds(0, 0);
    }
    else {
          setMotorSpeeds(-leftOutput, -rightOutput);
    }*/

    
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
  public void setMotorSpeeds(double leftOutput, double rightOutput){
    //var leftFeedforward = feedForward.calculate(speeds.leftMetersPerSecond);
    //var rightFeedforward = feedForward.calculate(speeds.rightMetersPerSecond);
    //System.out.println(leftEncoder.getRate());
    //System.out.println("Left encoder: " + leftEncoder.getRate());
    //double leftOutput = m_leftPIDController.calculate(leftEncoder.getRate(), speeds.leftMetersPerSecond);
    //double rightOutput = m_rightPIDController.calculate(rightEncoder.getRate(), speeds.rightMetersPerSecond);
    SmartDashboard.putNumber("Left Output: ", leftOutput);
    SmartDashboard.putNumber("Right Output: ", rightOutput);
    SetLeft(leftOutput);
    SetRight(rightOutput);

    //leftMotorControllerGroup.set(leftOutput);
    //rightMotorControllerGroup.set(rightOutput);

    //leftMotorControllerGroup.set(leftOutput + leftFeedforward);
    //rightMotorControllerGroup.set(rightOutput + rightFeedforward);

    //setLeftVoltage(leftOutput + leftFeedforward);
    //setRightVoltage(rightOutput + rightFeedforward);
  }
  public double getRightVelocity() {
    return rightEncoder.getRate();
  }
  
  public double getLeftVelocity() {
    return leftEncoder.getRate();
  }
  
  public double getLeftEncoder() {
    return leftEncoder.getRaw();
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
    pigeon.setYaw(pose.getRotation().getDegrees());
    m_odometry.resetPosition(getHeadingRotation2d(), getLeftEncoder(), getRightEncoder(), pose);
  }

  public double getHeadingDegrees() {
    return pigeon.getYaw();
  }

  public Rotation2d getHeadingRotation2d() {
    return Rotation2d.fromDegrees(getHeadingDegrees());
  }

  public void resetOdometry(Pose2d pose){
    leftEncoder.reset();
    rightEncoder.reset();
    m_drivetrainSimulator.setPose(pose);
    m_odometry.resetPosition(
        gyro.getRotation2d(), leftEncoder.getDistance(), rightEncoder.getDistance(), pose);
  }

  public double EncoderTickstoMeters(double ticks){
    double ticksInoneInch = 4096f / (Math.PI * 6);
    return Units.inchesToMeters(ticks / ticksInoneInch);
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds(){
    return new DifferentialDriveWheelSpeeds(EncoderTickstoMeters(getLeftVelocity()), 
    EncoderTickstoMeters(getRightVelocity()));
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

    SmartDashboard.putNumber("Left Encoder Value Meters: ", getLeftEncoder());
    SmartDashboard.putNumber("Right Encoder Value Meters", getRightEncoder());
    //SmartDashboard.putNumber("Gyro Heading", getHeading());
    //SmartDashboard.putString("Robot Pose", m_field.getRobotPose().toString());
    setpigeon();
    updateOdometry();
    m_field.setRobotPose(m_odometry.getPoseMeters());
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
    
    ChassisSpeeds m_ChassisSpeeds = DriveTrainConstants.kDriveKinematics.toChassisSpeeds(getWheelSpeeds());
    m_drivetrainSimulator.setInputs(leftMotorControllerGroup.get() * RobotController.getInputVoltage(),
    rightMotorControllerGroup.get() * RobotController.getInputVoltage());
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
    m_simYaw += m_ChassisSpeeds.omegaRadiansPerSecond * 0.02;
    pigeon.getSimCollection().setRawHeading(-Units.radiansToDegrees(m_simYaw));
    //setOdometry(pose);
    m_field.setRobotPose(m_odometry.getPoseMeters());
    //System.out.println(getPose());
    //angle.set(50);
    //m_field.setRobotPose(getPose());
  }

  public void updateOdometry() {
    m_odometry.update(
        gyro.getRotation2d(), leftEncoder.getDistance(), rightEncoder.getDistance());
  }
  
  // Method to reset the encoder values
	public void resetEncoder() {
		leftEncoder.reset();
		rightEncoder.reset();
	}

	// Method to reset the spartan board gyro values
	public void resetGyro() {
		pigeon.setYaw(0);
    pigeon.setAccumZAngle(0);
	}

  public void SetLeft(double speed) {
		LeftTop.set(speed);
		LeftBottom1.set(speed);
		LeftBottom2.set(speed);
	}

	// Setting the right master Talon's speed to the given parameter
	public void SetRight(double speed) {
		RightTop.set(-speed); //in correct setting, but "software fix"		
    //System.out.println("Right output current: " + RightTop.getOutputCurrent());
		RightBottom1.set(-speed);
		RightBottom2.set(-speed);
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
