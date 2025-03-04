package frc.robot2025.testBindings;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib2202.builder.RobotContainer;
import frc.lib2202.subsystem.hid.HID_Subsystem;
import frc.robot2025.commands.ClimberSetPos;
import frc.robot2025.commands.ElevatorCalibrate;
import frc.robot2025.commands.EndEffectorPercent;
import frc.robot2025.commands.testElevatorVelComd;
import frc.robot2025.subsystems.Elevator_Subsystem;

public class ElevTest {
    public static void myBindings(HID_Subsystem dc) {
        Elevator_Subsystem elevator_Subsystem = RobotContainer.getSubsystem(Elevator_Subsystem.class);
        // get an xbox controller for the operator, or null
        CommandXboxController opr = (dc.Operator() instanceof CommandXboxController)
                ? (CommandXboxController) dc.Operator()
                : null;

        opr.x().whileTrue(new testElevatorVelComd(30.0));
        opr.a().onTrue(new ElevatorCalibrate(-30.0));

        opr.y().onTrue(new ClimberSetPos(0.0));
        // opr.b().onTrue(new InstantCommand(() -> {
        // elevator_Subsystem.setHeight(50.0);
        // }));
        // opr.a().onTrue(new InstantCommand(() -> {
        // elevator_Subsystem.setHeight(90.0);
        // }));
        opr.povDown().onTrue(new InstantCommand(() -> {
            elevator_Subsystem.setHeight(46.0);
        }));
        opr.povLeft().onTrue(new InstantCommand(() -> {
            elevator_Subsystem.setHeight(86.0);
        }));
        opr.povRight().onTrue(new InstantCommand(() -> {
            elevator_Subsystem.setHeight(0.0);
        }));
        // opr.rightTrigger().onTrue(new InstantCommand(() -> {
        // elevator_Subsystem.setHeight(75.0);
        // }));
        opr.rightBumper().whileTrue(new EndEffectorPercent(-0.7, "rightBumper")); // reverse
        opr.rightTrigger().whileTrue(new EndEffectorPercent(.3, "rightTrigger"));

    }
}
