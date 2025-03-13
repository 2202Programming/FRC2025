package frc.robot2025;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib2202.builder.RobotContainer;
import frc.lib2202.command.swerve.RotateUntilSeeTags;
import frc.robot2025.Constants.Tag_Pose;
import frc.robot2025.commands.CoralPlaceSequence;
import frc.robot2025.commands.PickupSequence;
import frc.robot2025.subsystems.Elevator_Subsystem;
import frc.robot2025.subsystems.Elevator_Subsystem.Levels;

/*
 * Place commands named in PathPlaner autos here.
 */
public class RegisteredCommands {

    //Timeouts allow paths to continue in auto even if we miss a Note.   
    static final double ShooterTimeOut = 3.0;

    public static SendableChooser<Command> RegisterCommands() {
        SendableChooser<Command> autoChooser;
        final Elevator_Subsystem elevator_Subsystem = RobotContainer.getSubsystem(Elevator_Subsystem.class);
        NamedCommands.registerCommand("RotateTo", 
                new RotateUntilSeeTags(Tag_Pose.ID4, Tag_Pose.ID7));
        NamedCommands.registerCommand("L1Place",
                new InstantCommand(() -> {
                elevator_Subsystem.setHeight(0.0);
            }));
        NamedCommands.registerCommand("L2Place",
            new InstantCommand(() -> {
            elevator_Subsystem.setHeight(46.0);
        }));
        NamedCommands.registerCommand("L3Place",
            new InstantCommand(() -> {
            elevator_Subsystem.setHeight(86.0);
        }));
        NamedCommands.registerCommand("L4Place",
            new InstantCommand(() -> {
            elevator_Subsystem.setHeight(150.0);
        }));
        NamedCommands.registerCommand("Pickup",
            new InstantCommand(() -> {
            elevator_Subsystem.setHeight(50.0);
        }));
        NamedCommands.registerCommand("Release", 
            new CoralPlaceSequence(83.0)
        );
        NamedCommands.registerCommand("pickup station", 
        new PickupSequence(Levels.LOne, true)
        );



        autoChooser = AutoBuilder.buildAutoChooser();
        // select our auto
        SmartDashboard.putData("Auto Chooser", autoChooser);
        return autoChooser;
    }
}