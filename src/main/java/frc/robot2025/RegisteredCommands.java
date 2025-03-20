package frc.robot2025;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib2202.builder.RobotContainer;
import frc.lib2202.command.swerve.RotateUntilSeeTags;
import frc.robot2025.Constants.Tag_Pose;
import frc.robot2025.commands.DropSequenceBaseCommands.setWristPos;
import frc.robot2025.commands.DriveToReefTag;
import frc.robot2025.commands.PickupAdjustment;
import frc.robot2025.commands.GroundIntake.PlaceSequence;
import frc.robot2025.subsystems.Elevator_Subsystem;
import frc.robot2025.commands.DropSequenceBaseCommands.ReleaseCoral;
import frc.robot2025.commands.DropSequenceBaseCommands.setElevatorSetpoint;
import frc.robot2025.subsystems.Elevator_Subsystem.Levels;

/*
 * Place commands named in PathPlaner autos here.
 */

public class RegisteredCommands {
    final static Elevator_Subsystem elevator_Subsystem = RobotContainer.getSubsystem(Elevator_Subsystem.class);
    private static SequentialCommandGroup place(Levels level){
        return new SequentialCommandGroup (
            new setElevatorSetpoint(level),
            new setWristPos(true),
            new ReleaseCoral(),
            new setWristPos(false),
            new InstantCommand(() -> {
                elevator_Subsystem.setHeight(Levels.PickUp);
            }));
    }

    public static SendableChooser<Command> RegisterCommands() {
        SendableChooser<Command> autoChooser;

        NamedCommands.registerCommand("RotateTo", new RotateUntilSeeTags(Tag_Pose.ID4, Tag_Pose.ID7));
        NamedCommands.registerCommand("Pickup",   new InstantCommand(() -> {
            elevator_Subsystem.setHeight(Levels.PickUp); }));
        NamedCommands.registerCommand("PlaceL4", place(Levels.LFour));
        NamedCommands.registerCommand("PlaceL3", place(Levels.LThree));
        NamedCommands.registerCommand("PlaceL2", place(Levels.LTwo));
        NamedCommands.registerCommand("PlaceL1", place(Levels.LOne));
        NamedCommands.registerCommand("PickupAdjustment", new PickupAdjustment());
        NamedCommands.registerCommand("Release", new PlaceSequence("coral", 83.0 ));
        NamedCommands.registerCommand("DriveToReefTag", new DriveToReefTag(DriverStation.getAlliance().toString()));

        //enable chooser - builds autochooser list
        autoChooser = AutoBuilder.buildAutoChooser();
        // select our auto
        SmartDashboard.putData("Auto Chooser", autoChooser);
        return autoChooser;
    }
}