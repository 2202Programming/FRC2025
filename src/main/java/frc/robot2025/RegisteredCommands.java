package frc.robot2025;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

//import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib2202.builder.RobotContainer;
import frc.lib2202.command.swerve.RotateUntilSeeTags;
import frc.robot2025.Constants.Tag_Pose;
import frc.robot2025.commands.DropSequenceBaseCommands.setWristPos;
import frc.robot2025.commands.DriveToPickupTag;
import frc.robot2025.commands.DriveToReefTag;
import frc.robot2025.commands.PickupAdjustment;
import frc.robot2025.commands.GroundIntake.PlaceSequence;
import frc.robot2025.subsystems.Elevator_Subsystem;
import frc.robot2025.subsystems.WristFLA;
import frc.robot2025.commands.DropSequenceBaseCommands.ReleaseCoral;
import frc.robot2025.commands.DropSequenceBaseCommands.setElevatorSetpoint;
import frc.robot2025.subsystems.Elevator_Subsystem.Levels;

/*
 * Place commands named in PathPlaner autos here.
 */

public class RegisteredCommands {
    final static Elevator_Subsystem elevator_Subsystem = RobotContainer.getSubsystem(Elevator_Subsystem.class);
    private static SequentialCommandGroup place(Levels level){
        String name = (level == Levels.LTwo) ? "L2" : "L3";
        return new SequentialCommandGroup (
            new ParallelCommandGroup(
                new setElevatorSetpoint(level),
                new setWristPos(WristFLA.MID_POSITION, name)),
            new ReleaseCoral(),
            new InstantCommand(() -> {
                    elevator_Subsystem.setHeight(Levels.PickUp);
                }),
            new setWristPos(WristFLA.PICKUP_POSITION, "pickup"));
    }
    private static SequentialCommandGroup place4(Levels level){
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
            new setElevatorSetpoint(Levels.LFour).withTimeout(3.0)),
            new setWristPos(0.6, "L4"), //position for L4 drop
            new ReleaseCoral(),
            new setWristPos(WristFLA.PICKUP_POSITION, "pickup").withTimeout(1.0),
            new setElevatorSetpoint(Levels.PickUp));
    }

    public static SendableChooser<Command> RegisterCommands() {
        SendableChooser<Command> autoChooser;

        NamedCommands.registerCommand("RotateTo", new RotateUntilSeeTags(Tag_Pose.ID4, Tag_Pose.ID7));
        NamedCommands.registerCommand("Pickup",   new InstantCommand(() -> {
            elevator_Subsystem.setHeight(Levels.PickUp); }));
        NamedCommands.registerCommand("PlaceL4", place4(Levels.LFour));
        NamedCommands.registerCommand("PlaceL3", place(Levels.LThree));
        NamedCommands.registerCommand("PlaceL2", place(Levels.LTwo));
        NamedCommands.registerCommand("PlaceL1", place(Levels.LOne));
        NamedCommands.registerCommand("PickupAdjustment", new PickupAdjustment());
        NamedCommands.registerCommand("Release", new PlaceSequence("coral", 83.0 ));
        NamedCommands.registerCommand("DriveToReefTagRight", new DriveToReefTag("r"));
        NamedCommands.registerCommand("DriveToReefTagLeft", new DriveToReefTag("l"));
        NamedCommands.registerCommand("DriveToPickupTagLeft",new DriveToPickupTag("l"));
        NamedCommands.registerCommand("DriveToPickupTagRight",new DriveToPickupTag("r"));

        //enable chooser - builds autochooser list
        autoChooser = AutoBuilder.buildAutoChooser();
        // select our auto
        SmartDashboard.putData("Auto Chooser", autoChooser);
        return autoChooser;
    }
}