package frc.robot2025;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
//add when needed - import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib2202.builder.RobotContainer;
import frc.lib2202.command.swerve.AllianceAwareGyroReset;
import frc.lib2202.command.swerve.RobotCentricDrive;
import frc.lib2202.command.swerve.TargetCentricDrive;
import frc.lib2202.subsystem.swerve.SwerveDrivetrain;
import frc.robot2025.Constants.Tag_Pose;
import frc.lib2202.subsystem.hid.HID_Xbox_Subsystem;
import frc.lib2202.subsystem.hid.TMJoystickController;

/*
 * Please don't edit this without leads/mentor/driveteam review
 */
public final class BindingsCompetition {

    public static void ConfigureCompetition(HID_Xbox_Subsystem dc) {
        DriverBinding(dc);
        OperatorBindings(dc);
    }


    private static void DriverBinding(HID_Xbox_Subsystem dc) {
        var generic_driver = dc.Driver();
        var drivetrain = RobotContainer.getSubsystem(SwerveDrivetrain.class);

        // Driver Buttons depend on the type of controller drivers selects
        if (generic_driver instanceof TMJoystickController) {
            // Joystick
            TMJoystickController joystick = (TMJoystickController)generic_driver;
            joystick.trigger(TMJoystickController.ButtonType.TriggerButton).whileTrue(new RobotCentricDrive(drivetrain, dc));
            joystick.trigger(TMJoystickController.ButtonType.LeftOne).onTrue(new AllianceAwareGyroReset(true));
        } 
        else if (generic_driver instanceof CommandXboxController) {
            // XBox
            CommandXboxController driver = (CommandXboxController)generic_driver;
            driver.leftTrigger().whileTrue(new RobotCentricDrive(drivetrain, dc));
            driver.y().onTrue(new AllianceAwareGyroReset(true));
            driver.rightTrigger().whileTrue(new TargetCentricDrive(Tag_Pose.ID4, Tag_Pose.ID7));
        }
    }


    static void OperatorBindings(HID_Xbox_Subsystem dc) {
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
