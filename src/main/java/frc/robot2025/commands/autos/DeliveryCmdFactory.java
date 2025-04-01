package frc.robot2025.commands.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib2202.builder.RobotContainer;
import frc.lib2202.subsystem.OdometryInterface;
import frc.robot2025.commands.AlianceAwareSetPose;
import frc.robot2025.commands.DriveToPickupTag;
import frc.robot2025.commands.DriveToReefTag;
import frc.robot2025.commands.DropSequenceBaseCommands.ReleaseCoral;
import frc.robot2025.commands.DropSequenceBaseCommands.setElevatorSetpoint;
import frc.robot2025.commands.DropSequenceBaseCommands.setWristPos;
import frc.robot2025.subsystems.Elevator_Subsystem;
import frc.robot2025.subsystems.Elevator_Subsystem.Levels;
import frc.robot2025.subsystems.WristFLA;

/** Add your docs here. */
public class DeliveryCmdFactory {

    final OdometryInterface vpe;  //vision pose estimator
    final Elevator_Subsystem elevator; 
    final SendableChooser<Command> chooser;


    //Factory is initialized by getting correct subsystems
    public DeliveryCmdFactory(String vpeName) {
        vpe = RobotContainer.getSubsystemOrNull(vpeName);
        elevator = RobotContainer.getSubsystemOrNull(Elevator_Subsystem.class);
        chooser = RobotContainer.getRobotSpecs().getChooser();
    }

    public Command DeliverReef(String cmdName,
            Pose2d startPose, int reefPosition, String reefSide,  String pickupSide,
            Levels eleLevel, String levelTrimName, double wristPos) {
        
        SequentialCommandGroup cmd = new SequentialCommandGroup();
        cmd.setName(cmdName);
        @SuppressWarnings("unchecked")
        AlianceAwareSetPose initPose = new AlianceAwareSetPose(startPose, vpe::setPose);
        DriveToReefTag toReef = new DriveToReefTag(reefSide, reefPosition);
        DriveToPickupTag toPickup = new DriveToPickupTag(pickupSide);
        var eleCmd = ElevatorDelivery(eleLevel, levelTrimName, wristPos, 5);
        cmd.addCommands(initPose, toReef, eleCmd, toPickup);
        
        // add to our chooser
        chooser.addOption(cmdName, cmd);
        return cmd;
    }

    public Command ElevatorDelivery(Levels eleLevel, String levelTrimName, double wristPos, double releaseCount ) {
        if (elevator == null) return new PrintCommand("No elevator found.");
        // we have an elevator make a real command
        var cmd = new SequentialCommandGroup (
            //move elevator
            new ParallelCommandGroup(
                new setElevatorSetpoint(eleLevel, levelTrimName).withTimeout(2.0),
                new setWristPos(wristPos, levelTrimName)),
            //eject coral
            new ReleaseCoral(releaseCount),
            // return to pickup
            new ParallelCommandGroup(
                new setWristPos(WristFLA.PICKUP_POSITION, "pickup"),
                new setElevatorSetpoint(Levels.PickUp, "pickup")));
        
        return cmd;
    }

}
