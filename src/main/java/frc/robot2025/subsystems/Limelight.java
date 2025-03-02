// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2025.subsystems;

import static frc.lib2202.Constants.DEGperRAD;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Timer;
import frc.lib2202.builder.RobotContainer;
import frc.lib2202.subsystem.BaseLimelight;
import frc.lib2202.subsystem.swerve.IHeadingProvider;
import frc.robot2025.Constants.TheField;

public class Limelight extends BaseLimelight {
  // add any new variables not found in BaseLimelight
  protected NetworkTableEntry nt_bluepose2_x;
  protected NetworkTableEntry nt_bluepose2_y;

  // protected Pose2d megaPose;
  protected Pose2d bluePose2 = new Pose2d();
  protected final IHeadingProvider gyro;
  protected final int[] allTags;

  public Limelight() {
    this("limelight");
  }
  public Limelight(String name) {
    this(name, new Pose3d());  // 0,0,0... not great
  }
  
  public Limelight(String name, Pose3d position) {
    super(name);
    gyro = RobotContainer.getRobotSpecs().getHeadingProvider();

    //Set our camera's position on the robot
    var llRot =  position.getRotation(); 
    LimelightHelpers.setCameraPose_RobotSpace(name, 
      position.getX(), position.getY(), position.getZ(),
      llRot.getX()*DEGperRAD, llRot.getY()*DEGperRAD, llRot.getZ()*DEGperRAD);

    // these are "output" entries for user debugging
    nt_bluepose2_x = outputTable.getEntry("/LL Blue Pose2 X");
    nt_bluepose2_y = outputTable.getEntry("/LL Blue Pose2 Y");

    // save all the tags for reseting LL tag filter.
    var tgs = TheField.fieldLayout.getTags();
    allTags = new int[tgs.size()];
    for (int i=0; i< tgs.size(); i++)
      allTags[i] = tgs.get(i).ID;
  }


  // Some helper methods not in the base, candidates for moving to baseclass, but may be LL4
  public void setTargetTags(int[] tags) {
    LimelightHelpers.SetFiducialIDFiltersOverride(name, tags);
  }

  public void setTargetTagsAll() {
    LimelightHelpers.SetFiducialIDFiltersOverride(name, allTags);
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

    numAprilTags = 0; // gets changed if mt1 is valid
    // in sim, mt1 and mt2 are null, need to protect so we can debug
    if (mt1 != null &&
        mt1.tagCount == 1 &&
        mt1.rawFiducials.length == 1) // stricter criteria if only 1 tag seen
    {
      // rejectUpdate on poor conditions @Dr.J ?? what does [0] refer too???
      doRejectUpdate = (mt1.rawFiducials[0].ambiguity > 0.7) ||
          (mt1.rawFiducials[0].distToCamera > 3.0) ||
          (mt1.tagCount == 0) ||
          (Math.abs(gyro.getYawRate()) > 720.0); //reject if spining fast

      if (!doRejectUpdate) {
        bluePose = mt1.pose;
        bluePose2 = mt2.pose;        
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
