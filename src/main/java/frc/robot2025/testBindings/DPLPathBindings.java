package frc.robot2025.testBindings;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib2202.builder.RobotContainer;
import frc.lib2202.command.pathing.MoveToPose;
import frc.lib2202.subsystem.OdometryInterface;
import frc.lib2202.subsystem.hid.HID_Subsystem;
import frc.lib2202.subsystem.swerve.DriveTrainInterface;

public final class DPLPathBindings {

    static OdometryInterface odo;
    static DriveTrainInterface sdt;

    static void myBindings(HID_Subsystem dc) {
        odo = RobotContainer.getObjectOrNull("odometry");
        sdt = RobotContainer.getObjectOrNull("drivetrain");
        // get an xbox controller for the operator, or null
        CommandXboxController opr = (dc.Operator() instanceof CommandXboxController)
                ? (CommandXboxController) dc.Operator()
                : null;

        // if we have what we need, create our commands
        if (odo != null && sdt != null && opr != null) {
            xboxOperator(opr);
        }
    }

    static void xboxOperator(CommandXboxController opr) {
        // test moveToPose
        opr.povUp().whileTrue(new InstantCommand(() -> {
            Pose2d currentPose = odo.getPose();         //field coords
            // add 1m forward, field not robot.
            Pose2d target = new Pose2d(currentPose.getX() + 1.0,
                    currentPose.getY(), currentPose.getRotation());
            // calc path
            new MoveToPose(target).schedule();
        }));
    }

}
