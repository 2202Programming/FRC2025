package frc.robot2025;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
//add when needed - import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib2202.builder.RobotContainer;
import frc.lib2202.command.swerve.AllianceAwareGyroReset;
import frc.lib2202.command.swerve.RobotCentricDrive;
import frc.lib2202.subsystem.hid.HID_Xbox_Subsystem;
import frc.lib2202.subsystem.hid.TMJoystickController;
import frc.lib2202.subsystem.swerve.SwerveDrivetrain;
import frc.robot2025.commands.ElevatorMove;
import frc.robot2025.commands.testElevatorVelComd;
import frc.robot2025.commands.GroundIntake.AlgaePickupSequence;
import frc.robot2025.commands.GroundIntake.AlgaePlace;
import frc.robot2025.commands.GroundIntake.CoralPickupSequence;
import frc.robot2025.commands.GroundIntake.CoralPlace;
import frc.robot2025.subsystems.Elevator_Subsystem.Levels;

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
            @SuppressWarnings("unused")
            TMJoystickController joystick = (TMJoystickController)generic_driver;
        } 
        else if (generic_driver instanceof CommandXboxController) {
            // XBox
            CommandXboxController driver = (CommandXboxController)generic_driver;
            driver.leftTrigger().whileTrue(new RobotCentricDrive(drivetrain, dc));
            driver.y().onTrue(new AllianceAwareGyroReset(true));
           // driver.rightTrigger().whileTrue(new TargetCentricDrive(Tag_Pose.ID4, Tag_Pose.ID7));
            driver.x().whileTrue(new testElevatorVelComd(5.0));
            driver.a().whileTrue(new testElevatorVelComd(5.0));
            driver.y().whileTrue(new testElevatorVelComd(-5.0));
            driver.b().onTrue(new ElevatorMove(Levels.Ground, true));
            driver.rightTrigger().onTrue(new ElevatorMove(Levels.LCoral, true));
        }
    }


    static void OperatorBindings(HID_Xbox_Subsystem dc) {
        @SuppressWarnings("unused")
        var sideboard = dc.SwitchBoard();
        CommandXboxController operator = (CommandXboxController)dc.Operator();

        operator.a().whileTrue(new CoralPickupSequence());
        operator.b().whileTrue(new CoralPlace());
        operator.x().whileTrue(new AlgaePickupSequence());
        operator.y().whileTrue(new AlgaePlace());
        
        
        // Switchboard buttons too
        
        // Calibration commands

    }
}
