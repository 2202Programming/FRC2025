package frc.robot2025.testBindings;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib2202.builder.RobotContainer;
import frc.lib2202.command.pathing.MoveToPose;
import frc.lib2202.subsystem.OdometryInterface;
import frc.lib2202.subsystem.hid.HID_Subsystem;
import frc.lib2202.subsystem.swerve.DriveTrainInterface;
import frc.robot2025.commands.DriveToReefTag;
import frc.robot2025.subsystems.SignalLight;

public final class DPLPathTest {

    static OdometryInterface odo;
    static String OdometryName = "vision_odo"; //VisionPoseEstimator.class.getSimpleName();
    static DriveTrainInterface sdt;
    static SignalLight signal;

    public static void myBindings(HID_Subsystem dc) {
        odo = RobotContainer.getObjectOrNull(OdometryName);  // or "odometry"
        sdt = RobotContainer.getObjectOrNull("drivetrain");
        signal = RobotContainer.getObjectOrNull("signal");
        // get an xbox controller for the operator, or null
        CommandXboxController opr = (dc.Operator() instanceof CommandXboxController)
                ? (CommandXboxController) dc.Operator()
                : null;

        CommandXboxController driver = (dc.Driver() instanceof CommandXboxController)
                ? (CommandXboxController) dc.Driver()
                : null;

        // if we have what we need, create our commands
        // signal is protected in the bound command
        if (odo != null && sdt != null && opr != null && driver != null) {
            xboxOperator(opr);
            xboxDriver(driver);
        }
    }

    static void xboxOperator(CommandXboxController opr) {
        opr.a().onTrue(new InstantCommand( () ->{
            //reset position to blue corner, near 0,0
            Pose2d newPose = new Pose2d(0.45, 1.70, odo.getPose().getRotation());
            odo.setPose(newPose);
        }));


        // test moveToPose
        opr.povUp().onTrue(new InstantCommand(() -> {
            Pose2d currentPose = odo.getPose(); // field coords
            // add 1m forward, field not robot.
            Pose2d target = new Pose2d(currentPose.getX() + 1.0,
                    currentPose.getY(), currentPose.getRotation());
            // force a color
            if (signal != null)
                signal.setLight(SignalLight.Color.BLUE);
            // calc path
            Command cmd = new MoveToPose(OdometryName, target);
            // turn signal off after our move, if we have a signal object
            if (signal != null)
                cmd = cmd.andThen(signal.getColorCommand(SignalLight.Color.OFF));
            cmd.setName("moveto-fwd/w signal"); 
            cmd.schedule();
        }));

        opr.povDown().onTrue(new InstantCommand(() -> {
            Pose2d currentPose = odo.getPose(); // field coords
            // add 1m forward, field not robot.
            Pose2d target = new Pose2d(currentPose.getX() - 1.0,
                    currentPose.getY(), currentPose.getRotation());
            // calc path
            Command cmd = new MoveToPose(OdometryName, target);      
            cmd.setName("moveto-backup");     
            cmd.schedule();
        }));
    }

    static void xboxDriver(CommandXboxController driver) {
        // l/r determined by view from driver's station
        driver.leftTrigger().whileTrue(new DriveToReefTag("l"));
        driver.rightTrigger().whileTrue(new DriveToReefTag("r"));
    }

   
}