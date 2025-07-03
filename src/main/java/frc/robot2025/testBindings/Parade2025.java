// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2025.testBindings;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib2202.builder.RobotContainer;
import frc.lib2202.command.swerve.AllianceAwareGyroReset;
import frc.lib2202.command.swerve.RobotCentricDrive;
import frc.lib2202.subsystem.hid.HID_Subsystem;
import frc.lib2202.subsystem.swerve.DriveTrainInterface;
import frc.robot2025.commands.ElevatorCalibrate;
import frc.robot2025.commands.DropSequenceBaseCommands.ReleaseCoral;
import frc.robot2025.commands.DropSequenceBaseCommands.setElevatorSetpoint;
import frc.robot2025.commands.DropSequenceBaseCommands.setWristPos;
import frc.robot2025.subsystems.Elevator_Subsystem;
import frc.robot2025.subsystems.Elevator_Subsystem.Levels;
import frc.robot2025.subsystems.WristFLA;

/** Add your docs here. */
public class Parade2025 {
    public static void myBindings(HID_Subsystem dc) {
        CommandXboxController driver = (dc.Driver() instanceof CommandXboxController)
                ? (CommandXboxController) dc.Driver()
                : null;

        Elevator_Subsystem elevator = RobotContainer.getSubsystem(Elevator_Subsystem.class);

        DriveTrainInterface drivetrain = RobotContainer.getSubsystem("drivetrain");

        driver.leftTrigger().whileTrue(new RobotCentricDrive(drivetrain, dc));

        driver.rightTrigger().onTrue(new SequentialCommandGroup(
                new ReleaseCoral(),
                new setWristPos(1.5, "L4"),
                new ParallelCommandGroup(
                        new setWristPos(WristFLA.PICKUP_POSITION, "pickup"),
                        new setElevatorSetpoint(Levels.PickUp, "pickup"))));

        driver.povLeft().onTrue(new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new setElevatorSetpoint(80, "L2").withTimeout(2.0))));

        driver.povDown().onTrue(new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new setElevatorSetpoint(35, "L2").withTimeout(2.0))));
        
        driver.povUp().onTrue(new SequentialCommandGroup(
            new ParallelCommandGroup(
                new setElevatorSetpoint(110, "PL2").withTimeout(2.0))));
                        
        driver.b().onTrue(new ElevatorCalibrate(-30.0));
        // driver.a().onTrue(new SetZero());
        driver.y().onTrue(new AllianceAwareGyroReset());
    }
}
