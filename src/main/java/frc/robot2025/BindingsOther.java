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
import frc.robot2025.commands.GroundIntake.Debug.BtmArmBack;
import frc.robot2025.commands.GroundIntake.Debug.BtmArmFwd;
import frc.robot2025.commands.GroundIntake.Debug.BtmArmRelPos;
import frc.robot2025.commands.GroundIntake.Debug.TopArmBack;
import frc.robot2025.commands.GroundIntake.Debug.TopArmFwd;

/*
 * Bindings here for testing, 
 */
public class BindingsOther {

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
        CommandXboxController driver = (CommandXboxController)generic_driver;

        driver.leftBumper().whileTrue(new RobotCentricDrive(drivetrain, dc));
        driver.b().onTrue(new AllianceAwareGyroReset(false));

        //driver.rightBumper().onTrue(new BtmArmRelPos(2.5));
        //driver.leftBumper().onTrue(new BrmArmRelPos(-2.5));
    }

    static void OperatorBindings(HID_Xbox_Subsystem dc) {
        @SuppressWarnings("unused")
        CommandXboxController operator = (CommandXboxController)dc.Operator();

        operator.rightBumper().whileTrue(new BtmArmFwd());
        operator.leftBumper().whileTrue(new BtmArmBack());

        operator.povRight().whileTrue(new TopArmFwd());
        operator.povLeft().whileTrue(new TopArmBack());


    }

}
