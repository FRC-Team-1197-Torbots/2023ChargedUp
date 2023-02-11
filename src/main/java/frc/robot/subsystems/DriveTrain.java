// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.DriveTrainConstants;

public class DriveTrain extends SubsystemBase {

  private CANSparkMax LeftTop;
  private CANSparkMax LeftBottom1;
  private CANSparkMax LeftBottom2;

  private CANSparkMax RightTop;
  private CANSparkMax RightBottom1;
  private CANSparkMax RightBottom2;

  public static Solenoid solenoid;
  public static Compressor compressor;

  /** Creates a new ExampleSubsystem. */
  public DriveTrain() {
    //NavX = new Gyro(SPI.Port.kMXP);

    LeftTop = new CANSparkMax(DriveTrainConstants.LeftTopID, MotorType.kBrushless);
		LeftBottom1 = new CANSparkMax(DriveTrainConstants.LeftBottom1ID, MotorType.kBrushless);
		LeftBottom2 = new CANSparkMax(DriveTrainConstants.LeftBottom2ID, MotorType.kBrushless); 

		RightTop = new CANSparkMax(DriveTrainConstants.RightTopID, MotorType.kBrushless); 
		RightBottom1 = new CANSparkMax(DriveTrainConstants.RightBottom1ID, MotorType.kBrushless);		
		RightBottom2 = new CANSparkMax(DriveTrainConstants.RightBottom2ID, MotorType.kBrushless);

    solenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 0);
    compressor = new Compressor(PneumaticsModuleType.CTREPCM);

    //leftEncoder = new Encoder(5, 6, false, Encoder.EncodingType.k4X);
		//rightEncoder = new Encoder(7, 8, false, Encoder.EncodingType.k4X);

    //resetEncoder();
    //resetGyro();
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
  
  // Method to reset the encoder values
	public void resetEncoder() {
		//leftEncoder.reset();
		//rightEncoder.reset();
	}

	// Method to reset the spartan board gyro values
	public void resetGyro() {
		//gyro.reset(); 
	}

  public void SetLeft(double speed) {
		LeftTop.set(speed);
		LeftBottom1.set(-speed);
		LeftBottom2.set(-speed);
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
