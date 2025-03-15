// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2025.commands;

import edu.wpi.first.math.geometry.Pose2d;
import frc.lib2202.builder.RobotContainer;
import frc.lib2202.command.TargetWatcherCmd;
import frc.lib2202.util.PoseMath;
import frc.robot2025.Constants.TheField;
import frc.robot2025.subsystems.Limelight;


/** Add your docs here. */
public class distanceWatcher extends TargetWatcherCmd {

    private Pose2d targetPose;
    private Limelight m_Limelight = RobotContainer.getSubsystemOrNull("limelight");
    private int lastTargetID = 0;

    public distanceWatcher(Pose2d targetPose) {
        super();
        this.targetPose = targetPose;

    }

    public distanceWatcher() {
        super();
    }

    @Override
    public double getTargetDistance() {
        var tag = m_Limelight.getTID();

        if(tag != lastTargetID) {
            lastTargetID = tag;
            var optTarg = TheField.fieldLayout.getTagPose(tag);

            if(optTarg.isPresent()) {
                targetPose = optTarg.get().toPose2d();
            }

            System.out.println("distance watcher: new tag = " + lastTargetID);
        }

        return PoseMath.poseDistance(targetPose, m_Limelight.getBluePose());
    }

    @Override
    public void calculate() {

    }

    @Override
    public double getTargetAngle() {
        return lastTargetID;
    }

    @Override
    public double getTargetRPM() {
        return 0.0;
    }
}
