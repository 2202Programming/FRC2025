package frc.robot2025.testBindings;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib2202.builder.RobotContainer;
import frc.lib2202.subsystem.hid.HID_Subsystem;
import frc.robot2025.commands.ElevatorCalibrate;
import frc.robot2025.commands.EndEffectorPercent;
import frc.robot2025.commands.testElevatorVelComd;
import frc.robot2025.commands.DropSequenceBaseCommands.ReleaseCoral;
import frc.robot2025.commands.DropSequenceBaseCommands.setElevatorSetpoint;
import frc.robot2025.commands.DropSequenceBaseCommands.setWristPos;
import frc.robot2025.subsystems.Elevator_Subsystem;
import frc.robot2025.subsystems.Elevator_Subsystem.Levels;
import frc.robot2025.subsystems.Wrist;

public class ElevTest {
    public static void myBindings(HID_Subsystem dc) {
        Elevator_Subsystem elevator_Subsystem = RobotContainer.getSubsystem(Elevator_Subsystem.class);
        Wrist wrist = RobotContainer.getSubsystem(Wrist.class);
        // get an xbox controller for the operator, or null
        CommandXboxController opr = (dc.Operator() instanceof CommandXboxController)
                ? (CommandXboxController) dc.Operator()
                : null;

        opr.x().whileTrue(new testElevatorVelComd(30.0));
        opr.a().onTrue(new ElevatorCalibrate(-30.0));

        opr.b().onTrue(new SequentialCommandGroup (
            new ParallelCommandGroup(
                new setElevatorSetpoint(Levels.LTwo).withTimeout(5.0),
                new setWristPos(true)),
                new ReleaseCoral(),
                new ParallelCommandGroup(
                new setWristPos(false).withTimeout(0.5)),
                new setElevatorSetpoint(Levels.PickUp)
        ));
        opr.y().onTrue(new SequentialCommandGroup (
            new ParallelCommandGroup(
            new setElevatorSetpoint(Levels.LThree).withTimeout(2.0),
            new setWristPos(true)),
            new ReleaseCoral(),
            new ParallelCommandGroup(
            new setWristPos(false).withTimeout(0.5)),
            new setElevatorSetpoint(Levels.PickUp)
        ));
        opr.leftBumper().onTrue(new SequentialCommandGroup (
            new ParallelCommandGroup(
            new setElevatorSetpoint(Levels.LFour).withTimeout(3.0),
            new setWristPos(0.05)), //position for L4 drop
            new ReleaseCoral(),
            new setWristPos(false).withTimeout(1.0),
            new setElevatorSetpoint(Levels.PickUp)
        ));
        opr.leftBumper().onTrue(new InstantCommand(() -> {
            wrist.setPos(wrist.drop);
        }));
        opr.leftTrigger().onTrue(new InstantCommand(() -> {
            wrist.setPos(wrist.pickup);
        }));

        
        // opr.y().onTrue(new ClimberPosition(0.0));
        // opr.b().onTrue(new InstantCommand(() -> {
        // elevator_Subsystem.setHeight(50.0);
        // }));
        // opr.a().onTrue(new InstantCommand(() -> {
        // elevator_Subsystem.setHeight(90.0);
        // }));
        opr.povDown().onTrue(new InstantCommand(() -> {
            wrist.setPos(0.3);
        }));
        opr.povLeft().onTrue(new InstantCommand(() -> {
            wrist.setPos(0.05);
        }));
        opr.povRight().onTrue(new InstantCommand(() -> {
            elevator_Subsystem.setHeight(0.6);

        }));
        // opr.rightTrigger().onTrue(new InstantCommand(() -> {
        // elevator_Subsystem.setHeight(75.0);
        // }));
        opr.rightBumper().whileTrue(new EndEffectorPercent(-0.7, "rightBumper")); // reverse
        opr.rightTrigger().whileTrue(new EndEffectorPercent(.3, "rightTrigger"));

    }
}
