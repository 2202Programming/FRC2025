package frc.robot2025;

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

    // shared networktables that can be used for accessing shared globals 
    public final static class NTStrings {
      // defined in sensors
      public final static String NT_Name_Position = "EulerAngles";
    }
  
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
    public static final int CANDLE3 = 5;
    public static final int CANDLE4 = 6;

    // Ground Intake
    public static final int IntakeTop = 12; //TBD
    public static final int IntakeBtm = 13; //TBD
    public static final int IntakeWheel = 14; //TBD
    //CAN IDs
    public static final int CLIMBER = 10; //placeholder
    public static final int ELEVATOR_MAIN = 40; //placeholder
    public static final int ELEVATOR_FOLLOW = 41; //placeholder
    public static final int WRIST = 38; //placeholder
    public static final int END_EFFECTOR = 39; //placeholder

    // DT
    // synced as of 1/25/25 
    // https://docs.google.com/spreadsheets/d/1CyHzJscPIuvs0eFUY_qruQcuFui_w2nIXeXUyMwRKBU
// i tried putting them in numerical order but the IO tables are sorta inconsistant with ordering so nothing I can do
    public static final int BL_Angle = 20;
    public static final int BL_Drive = 21;
    public static final int BL_CANCoder = 28;

    public static final int FL_Angle = 23;
    public static final int FL_Drive = 22;
    public static final int FL_CANCoder = 29;

    public static final int BR_Angle = 25;
    public static final int BR_Drive = 24;
    public static final int BR_CANCoder = 31;

    public static final int FR_Angle = 26;
    public static final int FR_Drive = 27;
    public static final int FR_CANCoder = 30;

    // IMU
    public static final int PIGEON_IMU_CAN = 60;

  }

  public static final class PWM{

  }

  public static final class AnalogIn {
    // public static final int Pressure_Sensor = 0;
    // public static final int MAGAZINE_ANGLE = 0;
  }

  // pnumatics control module 1
  public static final class PCM1 {
    //public static final int Forward = 0;
    //public static final int Reverse = 1;
  }

  // pnumatics control module 2
  public static final class PCM2 {
  }

  public final class DigitalIO {
    public static final int EndEffector_Lightgate = 0;
    public static final int GroundIntakeLightGate = 0; // TBD 1/25/25 needs to be assigned
    

  }

  // TODO fix for 2025!!!!
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


}