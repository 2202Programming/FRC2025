// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2025.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Timer;
import frc.lib2202.builder.RobotContainer;
import frc.lib2202.subsystem.BaseLimelight;
import frc.lib2202.subsystem.swerve.IHeadingProvider;

public class Limelight extends BaseLimelight {
  // add any new variables not found in BaseLimelight
  protected NetworkTableEntry nt_bluepose2_x;
  protected NetworkTableEntry nt_bluepose2_y;

  // protected Pose2d megaPose;
  protected Pose2d bluePose2 = new Pose2d();
 
  protected final IHeadingProvider gyro;

  public Limelight() {
    this("limelight");
  }

  public Limelight(String name) {
    super(name);
    gyro = RobotContainer.getRobotSpecs().getHeadingProvider();

    // these are "output" entries for user debugging
    nt_bluepose2_x = outputTable.getEntry("/LL Blue Pose2 X");
    nt_bluepose2_y = outputTable.getEntry("/LL Blue Pose2 Y");
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    pipeline = pipelineNTE.getInteger(0);

    // LL apriltags stuff
    setRobotOrientationLL(); // runs once per frame for megatag2

    LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
    LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
    boolean doRejectUpdate = false;

    numAprilTags = 0; // gets chaged if mt1 is valid
    // in sim, mt1 and mt2 are null, need to protect so we can debug
    if (mt1 != null &&
        mt1.tagCount == 1 &&
        mt1.rawFiducials.length == 1) // stricter criteria if only 1 tag seen
    {
      // rejectUpdate on poor conditions @Dr.J ?? what does [0] refer too???
      doRejectUpdate = (mt1.rawFiducials[0].ambiguity > .7) ||
          (mt1.rawFiducials[0].distToCamera > 3) ||
          (mt1.tagCount == 0);

      if (!doRejectUpdate) {
        bluePose = mt1.pose;
        // @Dr.J - I think getHeading() is not an angular-velocity so this check realy
        // isn't doing anything.
        // TODO IF this is rate based, need to calc the yaw-rate from yaw samples
        if (Math.abs(gyro.getHeading().getDegrees()) < 720) // if our angular velocity is greater than 720 degrees per
                                                            // second, ignore vision updates
        {
          bluePose2 = mt2.pose;
        }
      }
      numAprilTags = mt1.tagCount;
    }

    // set output vars for accessors
    targetValid = (numAprilTags > 0);
    visionTimestamp = Timer.getFPGATimestamp()
        - (LimelightHelpers.getLatency_Pipeline(this.name) / 1000.0)
        - (LimelightHelpers.getLatency_Capture(this.name) / 1000.0);

    // update a few NT entries every frame
    nt_numApriltags.setInteger(numAprilTags);
    log(); // do logging at end
  }

  public void setRobotOrientationLL() {
    //Robot yaw in degrees. 0 = robot facing red alliance wall in FRC
    LimelightHelpers.SetRobotOrientation(name, gyro.getHeading().getDegrees(), 0, 0, 0, 0, 0);
  }

  @Override
  public void log() {
    if (log_counter % FRAME_MOD == 0) {
      super.log();  //handles log_counter++
      
      if (bluePose2 != null) {
        nt_bluepose2_x.setDouble(bluePose2.getX());
        nt_bluepose2_y.setDouble(bluePose2.getY());
      }
    }
  }
}
