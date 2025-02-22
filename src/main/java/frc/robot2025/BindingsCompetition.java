package frc.robot2025;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
//add when needed - import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib2202.builder.RobotContainer;
import frc.lib2202.command.swerve.AllianceAwareGyroReset;
import frc.lib2202.command.swerve.RobotCentricDrive;
import frc.lib2202.subsystem.hid.HID_Subsystem;
import frc.lib2202.subsystem.hid.TMJoystickController;
import frc.lib2202.subsystem.swerve.DriveTrainInterface;
import frc.robot2025.commands.ElevatorMove;
import frc.robot2025.commands.testElevatorVelComd;
import frc.robot2025.commands.GroundIntake.AlgaePickupSequence;
import frc.robot2025.commands.GroundIntake.AlgaePlace;
import frc.robot2025.commands.GroundIntake.CoralPickupSequence;
import frc.robot2025.commands.GroundIntake.CoralPlace;
import frc.robot2025.subsystems.GroundIntake;
import frc.robot2025.subsystems.Elevator_Subsystem.Levels;

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
            @SuppressWarnings("unused")
            TMJoystickController joystick = (TMJoystickController)generic_driver;

            // put Driver's joystick bindings here

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
            driver.b().onTrue(new ElevatorMove(Levels.Ground));
            driver.rightTrigger().onTrue(new ElevatorMove(Levels.LCoral));
        }
        else {
            DriverStation.reportWarning("Bindings: No driver bindings set, check controllers.", false);
        }
    }


    static void OperatorBindings(HID_Subsystem dc) {
        @SuppressWarnings("unused")
        var sideboard = dc.SwitchBoard();
        var generic_opr = dc.Operator();

        //buttons depend on what controller is plugged in
        if (generic_opr instanceof CommandXboxController) {
            CommandXboxController operator = (CommandXboxController)generic_opr;

            if(RobotContainer.getSubsystemOrNull(GroundIntake.class) != null) {
                operator.a().whileTrue(new CoralPickupSequence());
                operator.b().whileTrue(new CoralPlace());
                operator.x().whileTrue(new AlgaePickupSequence());
                operator.y().whileTrue(new AlgaePlace());
            }
        }
        else {
            DriverStation.reportWarning("Bindings: No operator bindings set, check controllers.", false);
        }
        
        // Switchboard buttons too
        
        // Calibration commands

    }
}
