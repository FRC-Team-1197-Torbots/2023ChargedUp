// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class DriveTrainConstants {
    public static final int LeftTopID = 1;
    public static final int LeftBottom1ID = 2;
    public static final int LeftBottom2ID = 3;

    public static final int RightTopID = 4;
    public static final int RightBottom1ID = 5;
    public static final int RightBottom2ID = 6;

    public static final double kTrackWidthMeters = Units.inchesToMeters(26.75);
    public static final double kGearRatio = 1;
    public static final double kWheelRadiusInches = 3;
    public static final double kLinearDistanceConversionFactor = (Units.inchesToMeters(1/(kGearRatio * 2 * Math.PI * Units.inchesToMeters(kWheelRadiusInches))*10)); // 1 / Circumference of wheel

  }

  public static class TeleopDriveConstants{
    public static final double STEER_SCALAR = 1.15;
    public static final double POSRANGE_MAX_ACCEL = 0.02;//0.03;//0.08;
    public static final double NEGRANGE_MAX_ACCEL = 0.03;//0.02;//0.03;
    public static final double MAX_DECEL = 0.03;//0.04;//0.06;
    public static final double MAX_VELOCITY = 50000f;//45000f; //28000 //39000 // 34,000 // 39,000
    
    public static final double velocitykP = 0.0000125;// velocity stuff probably not needed at all and should keep 0
    public static final double velocitykI = 0.0;
    public static final double velocitykD = 0;//0.0000008;

  }

  public static class AutoDriveConstants{
    public static final double ksVolts = 0.13606;
    public static final double kVoltSecondPerMeter = 2.2065;
    public static final double kVoltSecondSquaredPerMeter = 1.0313;

    public static final double kPDriveVel = 1.4051;
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(DriveTrainConstants.kTrackWidthMeters);

    public static final double kMaxVelocity = 3;
    public static final double kMaxAcceleration = 3;

    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
  }

  public static class AutoBalanceConstants{
    public static final double BalancekP = 0;
    public static final double BalancekI = 0;
    public static final double BalancekD = 0;
  }

  public static class ElevatorArmConstants{
    public static final int ElMotor1ID = 7;
    public static final int ElMotor2ID = 8;
    public static final int ArmID = 9;
    public static final int ClawID = 10;
    public static final int limitSwitch1Port = 0;
    public static final int limitSwitch2Port = 1;
    public static final int armSwitch1Port = 2;
    public static final int armSwitch2Port = 3;
    public static final double EL_IDLE_POSITION = 0;
    public static final double EL_INTAKE_POSITION = -4500;
    public static final double EL_GRAB_POSITION = 0;
    public static final double EL_STORE_POSITION = -9000;
    public static final double EL_MID_POSITION = -4500;
    public static final double EL_TOP_POSITION = -9000;

    public static final double ARM_TOP_POSITION = 45;
    public static final double ARM_MID_POSITION = 45;
    public static final double ARM_IDLE_POSITION = 0;
    public static final double ARM_STORE_POSITION = 120;
    public static final double ARM_INTAKE_POSITION = 45;
    public static final double ARM_GRAB_POSITION = 0;


    public enum STATE{
      IDLE, INTAKE, GRAB, STORE, MIDDLE, TOP
    }

    public enum TARGET{
      MIDDLE, TOP
    }

    public enum MoveElevatorArm{
      IDLE, ACTIVE
    }
    public enum GamePiece{
      CONE,CUBE
    }
  }
}
