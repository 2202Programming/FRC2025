package frc.robot2025.testBindings;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib2202.builder.RobotContainer;
import frc.lib2202.command.pathing.MoveToPose;
import frc.lib2202.subsystem.OdometryInterface;
import frc.lib2202.subsystem.hid.HID_Subsystem;
import frc.lib2202.subsystem.swerve.DriveTrainInterface;
import frc.robot2025.commands.ElevatorCalibrate;
import frc.robot2025.commands.EndEffectorPercent;
import frc.robot2025.commands.PickupAdjustment;
import frc.robot2025.commands.WristFLAToPos;
import frc.robot2025.commands.testElevatorVelComd;
import frc.robot2025.commands.DropSequenceBaseCommands.ReleaseCoral;
import frc.robot2025.commands.DropSequenceBaseCommands.setElevatorSetpoint;
import frc.robot2025.commands.DropSequenceBaseCommands.setWristPos;
import frc.robot2025.subsystems.Elevator_Subsystem;
// import frc.robot2025.subsystems.Elevator_Subsystem;
import frc.robot2025.subsystems.Elevator_Subsystem.Levels;
import frc.robot2025.subsystems.WristFLA;
import frc.robot2025.subsystems.SignalLight;
import frc.robot2025.subsystems.VisionPoseEstimator;

public class ElevTest {
    static OdometryInterface odo;
     static String OdometryName = VisionPoseEstimator.class.getSimpleName();
    static DriveTrainInterface sdt;
    static SignalLight signal;

    static Command move(){
        Pose2d currentPose = odo.getPose(); // field coords
            // add 1m forward, field not robot.
            Pose2d target = new Pose2d(currentPose.getX(),
                    currentPose.getY() + 0.127, currentPose.getRotation());
            // force a color
            if (signal != null)
                signal.setLight(SignalLight.Color.BLUE);
            // calc path
            Command cmd = new MoveToPose(OdometryName, target);
            if (signal != null)
            cmd = cmd.andThen(signal.getColorCommand(SignalLight.Color.OFF)); 
            return cmd;
    }
    public static void myBindings(HID_Subsystem dc) {
        Elevator_Subsystem elevator_Subsystem = RobotContainer.getSubsystem(Elevator_Subsystem.class);
        WristFLA wrist = RobotContainer.getSubsystem(WristFLA.class);
        // get an xbox controller for the operator, or null
        CommandXboxController opr = (dc.Operator() instanceof CommandXboxController)
                ? (CommandXboxController) dc.Operator()
                : null;

        opr.x().whileTrue(new testElevatorVelComd(30.0));
        opr.a().onTrue(new ElevatorCalibrate(-30.0));

        opr.b().onTrue(new SequentialCommandGroup (
            new ParallelCommandGroup(
                new InstantCommand(() -> {
                    Pose2d currentPose = odo.getPose(); // field coords
                    // add 1m forward, field not robot.
                    Pose2d target = new Pose2d(currentPose.getX(),
                            currentPose.getY() + 0.127, currentPose.getRotation());
                    // calc path
                    Command cmd = new MoveToPose(OdometryName, target);      
                    cmd.setName("moveto-backup");     
                    cmd.schedule();
                }),
                new setElevatorSetpoint(Levels.LTwo).withTimeout(5.0),
                new setWristPos(true)),
                new ReleaseCoral(),
                new ParallelCommandGroup(
                new setWristPos(false).withTimeout(0.5)),
                new setElevatorSetpoint(Levels.PickUp)
        ));
        opr.y().onTrue(new SequentialCommandGroup (
            new InstantCommand(() -> {
                Pose2d currentPose = odo.getPose(); // field coords
                // add 1m forward, field not robot.
                Pose2d target = new Pose2d(currentPose.getX(),
                        currentPose.getY() + 0.127, currentPose.getRotation());
                // calc path
                Command cmd = new MoveToPose(OdometryName, target);      
                cmd.setName("moveto-backup");     
                cmd.schedule();
            }),
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
                new InstantCommand(() -> {
                    Pose2d currentPose = odo.getPose(); // field coords
                    // add 1m forward, field not robot.
                    Pose2d target = new Pose2d(currentPose.getX(),
                            currentPose.getY() + 0.127, currentPose.getRotation());
                    // calc path
                    Command cmd = new MoveToPose(OdometryName, target);      
                    cmd.setName("moveto-backup");     
                    cmd.schedule();
                }),
            new setElevatorSetpoint(Levels.LFour).withTimeout(3.0)),
            new setWristPos(0.05), //position for L4 drop
            new ReleaseCoral(),
            new setWristPos(false).withTimeout(1.0),
            new setElevatorSetpoint(Levels.PickUp)
        ));
        opr.povDown().onTrue(new InstantCommand(() -> {
            wrist.setPosition(WristFLA.DROP_POSITION);
        }));

        opr.leftTrigger().onTrue(new InstantCommand(() -> {
            wrist.setPosition(WristFLA.PICKUP_POSITION);
        }));
        //just move robot
        opr.povUp().onTrue(new InstantCommand(() -> {
            Pose2d currentPose = odo.getPose(); // field coords
            // add 1m forward, field not robot.
            Pose2d target = new Pose2d(currentPose.getX(),
                    currentPose.getY() + 0.127, currentPose.getRotation());
            // force a color
            if (signal != null)
                signal.setLight(SignalLight.Color.BLUE);
            // calc path
            Command cmd = new MoveToPose(OdometryName, target);
            // turn signal off after our move, if we have a signal object
            if (signal != null)
                cmd = cmd.andThen(signal.getColorCommand(SignalLight.Color.OFF));
            cmd.setName("moveto-fwd/w signal"); 
            cmd.schedule();
        }));
        opr.povRight().onTrue(new WristFLAToPos(0));
        opr.povLeft().onTrue(new PickupAdjustment());
        opr.rightBumper().whileTrue(new EndEffectorPercent(-0.7, "rightBumper")); // reverse
        opr.rightTrigger().whileTrue(new EndEffectorPercent(.3, "rightTrigger"));

    }
}
        // opr.y().onTrue(new ClimberPosition(0.0));
        // opr.b().onTrue(new InstantCommand(() -> {
        // elevator_Subsystem.setHeight(50.0);
        // }));
        // opr.a().onTrue(new InstantCommand(() -> {
        // elevator_Subsystem.setHeight(90.0);
        // }));
        // opr.povDown().onTrue(new InstantCommand(() -> {
        //     wrist.setPosition(0.3);
        // }));
        // opr.povLeft().onTrue(new InstantCommand(() -> {
        //     wrist.setPosition(0.05);
        // }));
        // opr.povRight().onTrue(new InstantCommand(() -> {
        //     elevator_Subsystem.setHeight(0.6);
        // }));
        // }));
        // opr.rightTrigger().onTrue(new InstantCommand(() -> {
        // elevator_Subsystem.setHeight(75.0);
        // }));