// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2025;

//add when needed - import com.pathplanner.lib.config.RobotConfig;

import frc.lib2202.util.AprilTag2d;
/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  /*-------------------------Ports/CAN-------------------------------- */
  /**
   * CAN bus IDs
   * 
   * Please keep in order ID order
   * 
   */
  public static final class CAN {
    public static final int ROBORIO = 0;
    public static final int PDP = 1; // for rev
    public static final int PCM1 = 2; // for rev

    // lights
    public static final int CANDLE1 = 3;
    public static final int CANDLE2 = 4;

    // Drive Train IDs 20 - 31
    // drive train CAN addresses are set above with CANModuleConfig to support
    // different robots
    // See above CANModuleConfig definitions.
    //
    // Typically: Drv Ang CC Corner
    // -- --- -- ----
    // 20 21 31 BR
    // 22 23 28 BL
    // 24 25 29 FL
    // 26 27 30 FR
    //
    //
    // There are exceptions, check for your ROBOT.

   
    // IMU
    public static final int PIGEON_IMU_CAN = 60;


    // Whether to burn flash or not
    public static final boolean BURN_FLASH = false; // swerve-mk3
  }

  public static final class PWM{
    public static final int LEFT_AMP_MECHANISM = 1;
    public static final int RIGHT_AMP_MECHANISM = 0;
  }

  public static final class AnalogIn {
    public static final int Pressure_Sensor = 0;
    // public static final int MAGAZINE_ANGLE = 0;
  }

  // pnumatics control module 1
  public static final class PCM1 {
    public static final int Forward = 0;
    public static final int Reverse = 1;
  }

  // pnumatics control module 2
  public static final class PCM2 {
  }

  public final class DigitalIO {
    // placeholding in case we need. no confirmed subsystems yet- er
  }

  /*-- Apriltags -- */
  public static final class Tag_Pose {
    public static final AprilTag2d ID0 = new AprilTag2d(0, 0.0,0.0); //dont use tag ID 0, placeholder for array
    /**Blue source right */
    public static final AprilTag2d ID1 = new AprilTag2d(1, 15.079472, 0.245872);
    /**Blue source left */
    public static final AprilTag2d ID2 = new AprilTag2d(2, 16.185134, 0.883666);
    /**Red speaker right */
    public static final AprilTag2d ID3 = new AprilTag2d(3, 16.579342, 4.982718);
    /**Red speaker left */
    public static final AprilTag2d ID4 = new AprilTag2d(4, 16.579342, 5.547868);
    /**Red amp */
    public static final AprilTag2d ID5 = new AprilTag2d(5, 14.700758, 8.2042);
    /**Blue amp */
    public static final AprilTag2d ID6 = new AprilTag2d(6, 1.8415, 8.2042);
    /**Blue speaker right */
    public static final AprilTag2d ID7 = new AprilTag2d(7, 0.0381, 5.547868);
    /**Blue speaker left */
    public static final AprilTag2d ID8 = new AprilTag2d(8, 0.0381, 4.982718);
    /**Red source right */
    public static final AprilTag2d ID9 = new AprilTag2d(9, 0.356108, 0.883666);
    /**Red source left */
    public static final AprilTag2d ID10 = new AprilTag2d(10, 1.461516, 0.245872);
    /**Red stage (counter-clockwse starting at Stage Left) */
    public static final AprilTag2d ID11 = new AprilTag2d(11, 11.904726, 3.713226);
    public static final AprilTag2d ID12 = new AprilTag2d(12, 11.904726, 4.49834);
    public static final AprilTag2d ID13 = new AprilTag2d(13, 11.220196, 4.105148);
    /**Blue state (counter-clockwise starting at Center Stage) */
    public static final AprilTag2d ID14 = new AprilTag2d(14, 5.320792, 4.105148);
    public static final AprilTag2d ID15 = new AprilTag2d(15, 4.641342, 4.49834);
    public static final AprilTag2d ID16 = new AprilTag2d(16, 4.641342, 3.713226);

    public static AprilTag2d[] tagLocations = {ID0, ID1, ID2, ID3, ID4, ID5, ID6, ID7, ID8, ID9, ID10, ID11, ID12,
                                      ID13, ID14, ID15, ID16};     
  }

  /*-------NT------- */
  public final static class NTStrings {
    public final static String NT_Name_Position = "Position";
  }

}