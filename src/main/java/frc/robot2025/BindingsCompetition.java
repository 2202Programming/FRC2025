package frc.robot2025;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
//add when needed - import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib2202.builder.RobotContainer;
import frc.lib2202.command.swerve.AllianceAwareGyroReset;
import frc.lib2202.command.swerve.RobotCentricDrive;
import frc.lib2202.subsystem.hid.HID_Subsystem;
import frc.lib2202.subsystem.hid.TMJoystickController;
import frc.lib2202.subsystem.swerve.DriveTrainInterface;
import frc.robot2025.commands.EndEffectorPercent;
import frc.robot2025.commands.ScaleDriver;
import frc.robot2025.commands.GroundIntake.BtmArmVel;
import frc.robot2025.commands.GroundIntake.PickupSequence;
import frc.robot2025.commands.GroundIntake.PlaceSequence;
import frc.robot2025.commands.GroundIntake.SetZero;
import frc.robot2025.commands.GroundIntake.SpinRollers;
import frc.robot2025.commands.GroundIntake.TopArmVel;
import frc.robot2025.subsystems.Elevator_Subsystem;
import frc.robot2025.subsystems.EndEffector_Subsystem;
import frc.robot2025.subsystems.GroundIntake;
import frc.robot2025.subsystems.Wrist;

/*
 * Please don't edit this without leads/mentor/driveteam review
 */

public final class BindingsCompetition {

    public static void ConfigureCompetition(HID_Subsystem dc) {
        ConfigureCompetition(dc, true);
    }

    // optional disable opr binding for testing
    public static void ConfigureCompetition(HID_Subsystem dc, boolean initOpr) {
        DriverBinding(dc);
        if (initOpr) OperatorBindings(dc);
    }

    private static void DriverBinding(HID_Subsystem dc) {
        var generic_driver = dc.Driver();
        DriveTrainInterface drivetrain = RobotContainer.getSubsystem("drivetrain");

        // Driver Buttons depend on the type of controller drivers selects
        if (generic_driver instanceof TMJoystickController) {
            // Joystick
            @SuppressWarnings("unused")
            TMJoystickController joystick = (TMJoystickController) generic_driver;

            // put Driver's joystick bindings here

        } else if (generic_driver instanceof CommandXboxController) {
            // XBox
            CommandXboxController driver = (CommandXboxController) generic_driver;
            driver.rightTrigger().whileTrue(new RobotCentricDrive(drivetrain, dc));
            driver.y().onTrue(new AllianceAwareGyroReset(true));

            // Driver will wants precision robot-centric throttle drive on left trigger
            driver.leftTrigger().whileTrue(new ParallelCommandGroup(
                    new ScaleDriver(0.3),
                    new RobotCentricDrive(drivetrain, dc)));
        } else {
            DriverStation.reportError("Comp Bindings: No driver bindings set, check controllers.", false);
        }
    }

    static void OperatorBindings(HID_Subsystem dc) {
        var sideboard = dc.SwitchBoard();
        var generic_opr = dc.Operator();
        final Elevator_Subsystem elevator = RobotContainer.getSubsystem(Elevator_Subsystem.class);

        // buttons depend on what controller is plugged in
        if (generic_opr instanceof CommandXboxController) {

            CommandXboxController operator = (CommandXboxController) generic_opr;

            Trigger Cal = sideboard.sw11();
            Trigger NotCal = Cal.negate(); // regular competition mode

            // TODO sequence eventaully, TELL ELENA TO CHANGE once sequence is ready.
            operator.povDown().onTrue(new InstantCommand(() -> {
                elevator.setHeight(46.25); // l2
            })); // seriously, tell me once its changed
            NotCal.and(operator.povLeft()).onTrue(new InstantCommand(() -> {
                elevator.setHeight(87.25); // l3
            }));
            // TODO change value once mechanical adds more height
            operator.povUp().onTrue(new InstantCommand(() -> {
                elevator.setHeight(152.0);
            }));

            if (RobotContainer.getSubsystemOrNull(GroundIntake.class) != null) {
                NotCal.and(operator.a()).whileTrue(new PickupSequence("coral"));
                NotCal.and(operator.b()).whileTrue(new PlaceSequence("coral"));
                operator.x().whileTrue(new PickupSequence("algae"));
                operator.y().whileTrue(new PlaceSequence("algae"));
            }
            if (RobotContainer.getSubsystemOrNull(Elevator_Subsystem.class) != null) {
                /*
                 * From drive team
                 * operator.povUp().onTrue(); //high
                 * NotCal.and(operator.povLeft()).onTrue(); //mid
                 * operator.povDown().onTrue(); //low
                 * NotCal.and(operator.povRight()).onTrue(); //intake height
                 */
            }
            if (RobotContainer.getSubsystemOrNull(EndEffector_Subsystem.class) != null) {
                // TODO change to rpm, i just plucked these values off so i have no clue if
                // they're viable -er
                NotCal.and(operator.rightBumper()).whileTrue(new EndEffectorPercent(-.3, "rightBumper")); // reverse
                operator.rightTrigger().whileTrue(new EndEffectorPercent(.5, "rightTrigger")); //
            }
            if (RobotContainer.getSubsystemOrNull(Wrist.class) != null) {
            }

            //Calibration
            Cal.and(operator.rightBumper()).whileTrue(new BtmArmVel(30.0));
            Cal.and(operator.leftBumper()).whileTrue(new BtmArmVel(-30.0));
            Cal.and(operator.povRight()).whileTrue(new TopArmVel(30.0));
            Cal.and(operator.povLeft()).whileTrue(new TopArmVel(-30.0));
            Cal.and(operator.b()).onTrue(new SetZero());
            Cal.and(operator.a()).whileTrue(new SpinRollers(15.0));
            
        }

        else {
            DriverStation.reportWarning("Comp Bindings: No operator bindings set, check controllers.", false);
        }

        // Switchboard buttons too

        // Calibration commands

    }
}
