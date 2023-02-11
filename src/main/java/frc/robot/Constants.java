// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
}
