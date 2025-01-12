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
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib2202.builder.RobotContainer;
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
        Competition,  DriveTest,
    }

    static Bindings bindings = Bindings.DriveTest;

    public static void ConfigureOther(HID_Xbox_Subsystem dc) {
        DriverBinding(dc);
        OperatorBindings(dc);
    }

    // wrap the pathloading with try/catch
    static PathPlannerPath loadFromFile(String pathName) {
        try{
            // Load the path you want to follow using its name in the GUI
            return PathPlannerPath.fromPathFile(pathName);
        } catch (Exception e) {
            DriverStation.reportError("Big oops loading pn="+ pathName + ":" + e.getMessage(), e.getStackTrace());
            return null;
        }
    }



    static void DriverBinding(HID_Xbox_Subsystem dc) {
        var generic_driver = dc.Driver();
        var drivetrain = RobotContainer.getSubsystem(SwerveDrivetrain.class);
        
        //TODO - handle paths better, maybe move configAutoBuilder out of swerve code
        PathPlannerPath pathBlue1 = loadFromFile("test_1m");//"blue1");
        PathPlannerPath pathRed1 = loadFromFile("test_1m");//"red1");
        PathPlannerPath pathTest_1m = loadFromFile("test_1m");

        switch (bindings) {

            case DriveTest:
                // deal with xbox or joystick controller for driver
                if (generic_driver instanceof CommandXboxController) {
                    CommandXboxController driver = (CommandXboxController)generic_driver;
            
                driver.leftBumper().whileTrue(new RobotCentricDrive(drivetrain, dc));
                driver.b().onTrue(new AllianceAwareGyroReset(false));

                // This appears to break if initial pose is too close to path start pose
                // (zero-length path?)
                if (pathBlue1 != null)
                    driver.x().onTrue(new SequentialCommandGroup(
                        new InstantCommand(drivetrain::printPose),
                        AutoBuilder.pathfindThenFollowPath(pathBlue1,
                                new PathConstraints(3.0, 3.0,
                                        Units.degreesToRadians(540),
                                        Units.degreesToRadians(720))),
                        new InstantCommand(drivetrain::printPose)));
                if (pathRed1 != null)
                    driver.b().onTrue(new SequentialCommandGroup(
                        new InstantCommand(drivetrain::printPose),
                        AutoBuilder.pathfindThenFollowPath(pathRed1,
                                new PathConstraints(3.0, 3.0,
                                        Units.degreesToRadians(540),
                                        Units.degreesToRadians(720))),
                        new InstantCommand(drivetrain::printPose)));

                // Start any watcher commands
               
                // This appears to break if initial pose is too close to path start pose
                // (zero-length path?)
                if (pathTest_1m != null)
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

                }
                else {
                    // put driver TMjoystick commands here
                }
                break;

            default:
                break;
        }
    }

    static void OperatorBindings(HID_Xbox_Subsystem dc) {
        @SuppressWarnings("unused")
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
