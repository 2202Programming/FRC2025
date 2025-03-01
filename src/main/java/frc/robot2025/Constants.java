package frc.robot2025;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;

import frc.lib2202.util.AprilTag2d;  //TODO - prefer WPILIB class, deprecate

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
    public static final int CANDLE3 = 5;
    public static final int CANDLE4 = 6;

    // Ground Intake
    public static final int IntakeTop = 51;
    public static final int IntakeBtm = 52; 
    public static final int IntakeWheel = 50; 
    //CAN IDs
    public static final int CLIMBER = 10; //placeholder
    public static final int ELEVATOR_MAIN = 40; //placeholder
    public static final int ELEVATOR_FOLLOW = 41; //placeholder
    public static final int WRIST = 38; //placeholder
    public static final int END_EFFECTOR = 39; //placeholder

    // DT
    // synced as of 1/25/25 
    // https://docs.google.com/spreadsheets/d/1CyHzJscPIuvs0eFUY_qruQcuFui_w2nIXeXUyMwRKBU
    //
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
    public static final int Wrist = 0;
  }

  public static final class AnalogIn {
  }

  // pnumatics control module 1
  public static final class PCM1 {
  }

  // pnumatics control module 2
  public static final class PCM2 {
  }

  public final class DigitalIO {
    //TODO - fix these values once wired TODO update/check with google docs
    public static final int GroundIntakeHasCoral = 0; 
    public static final int GroundIntakeHasAlgae = 1;
    public static final int END_EFFECTOR_WHEEL_LOW_LIGHTGATE = 2; 
    public static final int END_EFFECTOR_LOAD_HIGH_LIGHTGATE = 3; 
    public static final int ElevatorZeroLS = 4;    
    
    public static final int SignalLight1 = 7;
    public static final int SignalLight2 = 8;
    public static final int SignalLight3 = 9;              
  }

  //The Field info use WPILIB data
  public class TheField {
    public static AprilTagFields fieldChoice = AprilTagFields.k2025ReefscapeWelded;
    public static AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(fieldChoice);

    public static int[] ReefIdsRed =  { 6,  7,  8,  9, 10, 11};
    public static int[] ReefIdsBlue = {17, 18, 19, 20, 21, 22};

    // Below tags are just examples for testing and to complete AprilTag2d to be deprecated
    // take care to use ids that exist, optional check not done here.
    public static Pose3d Tag4 = fieldLayout.getTagPose(4).get();
    public static Pose3d Tag7 = fieldLayout.getTagPose(7).get();    
  }
  
  // Tags called out in registered command - TODO use 3DPose
  public static final class Tag_Pose {
    // DON'T do this, only setup to get Registered command WIP working
    static AprilTag2d ID4 = new AprilTag2d(4, TheField.Tag4.getX(),TheField.Tag4.getY() );
    static AprilTag2d ID7 = new AprilTag2d(7, TheField.Tag7.getX(),TheField.Tag7.getY() );
  }

}