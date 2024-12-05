package frc.robot2025;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib2202.builder.RobotContainer;
import frc.lib2202.command.PDPMonitorCmd;
import frc.lib2202.command.swerve.AllianceAwareGyroReset;
import frc.lib2202.command.swerve.RobotCentricDrive;
import frc.lib2202.command.swerve.TargetCentricDrive;
import frc.lib2202.command.swerve.calibrate.TestConstantVelocity;
import frc.lib2202.command.swerve.calibrate.TestRotateVelocity;
import frc.lib2202.subsystem.hid.HID_Xbox_Subsystem;
import frc.lib2202.subsystem.swerve.SwerveDrivetrain;
import frc.robot2025.Constants.Tag_Pose;

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

        switch (bindings) {

            case DriveTest:
                driver.leftBumper().whileTrue(new RobotCentricDrive(drivetrain, dc));
                driver.b().onTrue(new AllianceAwareGyroReset(false));

                // This appears to break if initial pose is too close to path start pose
                // (zero-length path?)
                driver.x().onTrue(new SequentialCommandGroup(
                        new InstantCommand(drivetrain::printPose),
                        AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromPathFile("blue1"),
                                new PathConstraints(3.0, 3.0,
                                        Units.degreesToRadians(540),
                                        Units.degreesToRadians(720))),
                        new InstantCommand(drivetrain::printPose)));

                driver.b().onTrue(new SequentialCommandGroup(
                        new InstantCommand(drivetrain::printPose),
                        AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromPathFile("red1"),
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
                        AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromPathFile("test_1m"),
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
