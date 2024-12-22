package frc.robot2025;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib2202.builder.RobotContainer;
import frc.lib2202.command.PDPMonitorCmd;
import frc.lib2202.command.swerve.AllianceAwareGyroReset;
import frc.lib2202.command.swerve.RobotCentricDrive;
import frc.lib2202.subsystem.hid.HID_Xbox_Subsystem;
import frc.lib2202.subsystem.swerve.SwerveDrivetrain;

/*
 * Bindings here for testing, 
 */
public class BindingsOther {

    // enum for bindings add when needed
    public enum Bindings {
        Competition,
        DriveTest, Shooter_test, IntakeTesting, auto_shooter_test, new_bot_test, comp_not_comp, Etude
    }

    static Bindings bindings = Bindings.DriveTest;

    public static void ConfigureOther(HID_Xbox_Subsystem dc) {
        DriverBinding(dc);
        OperatorBindings(dc);
    }

    static void DriverBinding(HID_Xbox_Subsystem dc) {
        var driver = dc.Driver();

        var drivetrain = RobotContainer.getSubsystem(SwerveDrivetrain.class);

        PathPlannerPath pathBlue1;
    try{
        // Load the path you want to follow using its name in the GUI
        pathBlue1 = PathPlannerPath.fromPathFile("blue1");
        } catch (Exception e) {
        DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
        pathBlue1 = null;
    }

    PathPlannerPath pathRed1;
    try{
        // Load the path you want to follow using its name in the GUI
        pathRed1 = PathPlannerPath.fromPathFile("red1");
        } catch (Exception e) {
        DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
        pathRed1 = null;
    }

    PathPlannerPath pathTest_1m;
    try{
        // Load the path you want to follow using its name in the GUI
        pathTest_1m = PathPlannerPath.fromPathFile("test_1m");
        } catch (Exception e) {
        DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
        pathTest_1m = null;
    }
        switch (bindings) {

            case DriveTest:
                driver.leftBumper().whileTrue(new RobotCentricDrive(drivetrain, dc));
                driver.b().onTrue(new AllianceAwareGyroReset(false));

                // This appears to break if initial pose is too close to path start pose
                // (zero-length path?)
                driver.x().onTrue(new SequentialCommandGroup(
                        new InstantCommand(drivetrain::printPose),
                        AutoBuilder.pathfindThenFollowPath(pathBlue1,
                                new PathConstraints(3.0, 3.0,
                                        Units.degreesToRadians(540),
                                        Units.degreesToRadians(720))),
                        new InstantCommand(drivetrain::printPose)));

                driver.b().onTrue(new SequentialCommandGroup(
                        new InstantCommand(drivetrain::printPose),
                        AutoBuilder.pathfindThenFollowPath(pathRed1,
                                new PathConstraints(3.0, 3.0,
                                        Units.degreesToRadians(540),
                                        Units.degreesToRadians(720))),
                        new InstantCommand(drivetrain::printPose)));

                // Start any watcher commands
                new PDPMonitorCmd(); // auto scheduled, runs when disabled
                // This appears to break if initial pose is too close to path start pose
                // (zero-length path?)
                driver.a().onTrue(new SequentialCommandGroup(
                        new InstantCommand(drivetrain::printPose),
                        AutoBuilder.pathfindThenFollowPath(pathTest_1m,
                                new PathConstraints(3.0, 3.0, Units.degreesToRadians(540),
                                        Units.degreesToRadians(720))),
                        new InstantCommand(drivetrain::printPose)));

                driver.x().onTrue(new SequentialCommandGroup(
                        new InstantCommand(drivetrain::printPose),
                        AutoBuilder.pathfindToPose(new Pose2d(new Translation2d(1.73, 5.38), new Rotation2d(0.0)),
                                new PathConstraints(3.0, 3.0, Units.degreesToRadians(540),
                                        Units.degreesToRadians(720))),
                        new InstantCommand(drivetrain::printPose)));
                break;



            default:
                break;
        }
    }

    static void OperatorBindings(HID_Xbox_Subsystem dc) {
        var operator = dc.Operator();


        switch (bindings) {
            case Competition:
                BindingsCompetition.OperatorBindings(dc);
                break;

            default:
                break;
        }
    }

}
