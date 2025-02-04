package frc.robot2025;

import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib2202.builder.RobotContainer;
import frc.lib2202.command.swerve.AllianceAwareGyroReset;
import frc.lib2202.command.swerve.RobotCentricDrive;
import frc.lib2202.command.swerve.TargetCentricDrive;
import frc.lib2202.subsystem.swerve.DriveTrainInterface;
import frc.robot2025.Constants.Tag_Pose;
import frc.lib2202.subsystem.hid.HID_Subsystem;
import frc.lib2202.subsystem.hid.TMJoystickController;

/*
 * Please don't edit this without leads/mentor/driveteam review
 */
public final class BindingsCompetition {

    public static void ConfigureCompetition(HID_Subsystem dc) {
        DriverBinding(dc);
        OperatorBindings(dc);
    }


    private static void DriverBinding(HID_Subsystem dc) {
        var generic_driver = dc.Driver();
        DriveTrainInterface drivetrain = RobotContainer.getSubsystem("drivetrain");

        // Driver Buttons depend on the type of controller drivers selects
        if (generic_driver instanceof TMJoystickController) {
            // Joystick
            TMJoystickController joystick = (TMJoystickController)generic_driver;
            joystick.trigger(TMJoystickController.ButtonType.TriggerButton).whileTrue(new RobotCentricDrive());
            joystick.trigger(TMJoystickController.ButtonType.LeftOne).onTrue(new AllianceAwareGyroReset(true));
        } 
        else if (generic_driver instanceof CommandXboxController) {
            // XBox
            CommandXboxController driver = (CommandXboxController)generic_driver;
            driver.leftTrigger().whileTrue(new RobotCentricDrive(drivetrain, dc));
            driver.y().onTrue(new AllianceAwareGyroReset(true));
            driver.rightTrigger().whileTrue(new TargetCentricDrive(Tag_Pose.ID4, Tag_Pose.ID7));
        }
        else if (generic_driver instanceof CommandPS4Controller) {
            // XBox
            CommandPS4Controller driver = (CommandPS4Controller)generic_driver;
            driver.L2().whileTrue(new RobotCentricDrive(drivetrain, dc));
            driver.triangle().onTrue(new AllianceAwareGyroReset(true));
            driver.R2().whileTrue(new TargetCentricDrive(Tag_Pose.ID4, Tag_Pose.ID7, "limelight"));
        }
    }


    static void OperatorBindings(HID_Subsystem dc) {
        @SuppressWarnings("unused")
        var sideboard = dc.SwitchBoard();
        @SuppressWarnings("unused")
        var operator = dc.Operator();

        // Switchboard buttons too


        /***************************************************************************************/
        // REAL COMPETITION BINDINGS.

        
        // Calibration commands

    }
}
