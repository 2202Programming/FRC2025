package frc.robot2025.testBindings;

import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib2202.builder.RobotContainer;
import frc.lib2202.command.WatcherCmd;
import frc.lib2202.command.pathing.MoveToPose;
import frc.lib2202.subsystem.OdometryInterface;
import frc.lib2202.subsystem.hid.HID_Subsystem;
import frc.lib2202.subsystem.swerve.DriveTrainInterface;
import frc.robot2025.commands.DriveToReefTag;
import frc.robot2025.subsystems.SignalLight;

public final class DPLPathTest {

    static OdometryInterface odo;
    static DriveTrainInterface sdt;
    static SignalLight signal;

    public static void myBindings(HID_Subsystem dc) {
        odo = RobotContainer.getObjectOrNull("odometry");
        sdt = RobotContainer.getObjectOrNull("drivetrain");
        signal = RobotContainer.getObjectOrNull("signal");
        // get an xbox controller for the operator, or null
        CommandXboxController opr = (dc.Operator() instanceof CommandXboxController)
                ? (CommandXboxController) dc.Operator()
                : null;

        // if we have what we need, create our commands
        // signal is protected in the bound command
        if (odo != null && sdt != null && opr != null) {
            xboxOperator(opr);
        }
    }

    static void xboxOperator(CommandXboxController opr) {
        // test moveToPose
        opr.povUp().onTrue(new InstantCommand(() -> {
            Pose2d currentPose = odo.getPose(); // field coords
            // add 1m forward, field not robot.
            Pose2d target = new Pose2d(currentPose.getX() + 3.0,
                    currentPose.getY(), currentPose.getRotation());
            // force a color
            if (signal != null)
                signal.setLight(SignalLight.Color.BLUE);
            // calc path
            Command cmd = new MoveToPose(target);
            // turn signal off after our move, if we have a signal object
            if (signal != null)
                cmd = cmd.andThen(signal.getColorCommand(SignalLight.Color.OFF));
            cmd.setName("moveto-fwd/w signal"); 
            cmd.schedule();
        }));

        opr.povDown().onTrue(new InstantCommand(() -> {
            Pose2d currentPose = odo.getPose(); // field coords
            // add 1m forward, field not robot.
            Pose2d target = new Pose2d(currentPose.getX() - 3.0,
                    currentPose.getY(), currentPose.getRotation());
            // calc path
            Command cmd = new MoveToPose(target);      
            cmd.setName("moveto-backup");     
            cmd.schedule();
        }));


        opr.leftStick().whileTrue(new DriveToReefTag("l"));
        opr.rightStick().whileTrue(new DriveToReefTag("r"));

        new PathWatcher();
    }

    //TODO figure better spot for this
    public static class PathWatcher extends WatcherCmd {
    private final Field2d field;

    public PathWatcher(){
        field = new Field2d();
        SmartDashboard.putData("PathWatcher", field);

        // Logging callback for current robot pose
        PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
            // Do whatever you want with the pose here
            field.setRobotPose(pose);
        });

        // Logging callback for target robot pose
        PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
            // Do whatever you want with the pose here
            field.getObject("target pose").setPose(pose);
        });

        // Logging callback for the active path, this is sent as a list of poses
        PathPlannerLogging.setLogActivePathCallback((poses) -> {
            // Do whatever you want with the poses here
            field.getObject("path").setPoses(poses);
        });
    }

    @Override
    public void ntcreate() {
        
    }

    @Override
    public void ntupdate() {
        
 
       }
   }
}

