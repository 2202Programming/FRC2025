package frc.robot2025;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib2202.builder.RobotContainer;
import frc.robot2025.commands.DriveToPickupTag;
import frc.robot2025.commands.DriveToReefTag;
import frc.robot2025.commands.PickupAdjustment;
import frc.robot2025.commands.DropSequenceBaseCommands.ReleaseCoral;
import frc.robot2025.commands.DropSequenceBaseCommands.setElevatorSetpoint;
import frc.robot2025.commands.DropSequenceBaseCommands.setWristPos;
import frc.robot2025.commands.GroundIntake.PlaceSequence;
import frc.robot2025.subsystems.Elevator_Subsystem;
import frc.robot2025.subsystems.Elevator_Subsystem.Levels;
import frc.robot2025.subsystems.WristFLA;

/*
 * Place commands named in PathPlaner autos here.
 */

public class RegisteredCommands {
    final static Elevator_Subsystem elevator_Subsystem = RobotContainer.getSubsystem(Elevator_Subsystem.class);
    private static Command place(Levels level){
        String name = (level == Levels.LTwo) ? "L2" : "L3";
        return new SequentialCommandGroup (
            new ParallelCommandGroup(
                new setElevatorSetpoint(level, name),
                new setWristPos(WristFLA.MID_POSITION, name)),
            new ReleaseCoral(),
            new InstantCommand(() -> {
                    elevator_Subsystem.setHeight(Levels.PickUp);
                }),
            new setWristPos(WristFLA.PICKUP_POSITION, "pickup"));
    }
    private static Command place4(Levels level){
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
            new setElevatorSetpoint(Levels.LFour, "L4").withTimeout(3.0),
            new setWristPos(1.5, "L4")),
            new setWristPos(0.3, "L4"), //position for L4 drop
            new ReleaseCoral(),
            new setWristPos(1.5, "L4"),
            new ParallelCommandGroup(
            new setWristPos(WristFLA.PICKUP_POSITION, "pickup").withTimeout(1.0),
            new setElevatorSetpoint(Levels.PickUp, "pickup")));
    }

    public static void RegisterCommands() {
        NamedCommands.registerCommand("Pickup",   new InstantCommand(() -> {
            elevator_Subsystem.setHeight(Levels.PickUp); }));
        NamedCommands.registerCommand("PlaceL4", place4(Levels.LFour));
        NamedCommands.registerCommand("PlaceL3", place(Levels.LThree));
        NamedCommands.registerCommand("PlaceL2", place(Levels.LTwo));
        NamedCommands.registerCommand("PlaceL1", place(Levels.LOne));
        NamedCommands.registerCommand("PickupAdjustment", new PickupAdjustment());
        // TODO - need a real pickup here
        NamedCommands.registerCommand("WaitForPickup", new WaitCommand(2.0));  //TODO
        NamedCommands.registerCommand("Release", new PlaceSequence("coral", 83.0 ));
        NamedCommands.registerCommand("DriveToReefTagRight", new DriveToReefTag("r")
                                    .withDistanceScheduleCmd(place4(Levels.LFour), 1.6));
                                    // .withDistanceScheduleCmd(new PrintCommand("sched place .8m away"), 0.8));
        NamedCommands.registerCommand("DriveToReefTagLeft", new DriveToReefTag("l")
                                    .withDistanceScheduleCmd(place4(Levels.LFour), 1.6));
        NamedCommands.registerCommand("DriveToPickupTagLeft",new DriveToPickupTag("left"));
        NamedCommands.registerCommand("DriveToPickupTagRight",new DriveToPickupTag("right"));
    }
}