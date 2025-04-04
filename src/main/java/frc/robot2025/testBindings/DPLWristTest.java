package frc.robot2025.testBindings;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib2202.subsystem.hid.HID_Subsystem;
import frc.robot2025.commands.EndEffectorPercent;
import frc.robot2025.commands.DropSequenceBaseCommands.setWristPos;
import frc.robot2025.subsystems.WristFLA;

//quick test to drive just the wrist
public class DPLWristTest {
    public static void myBindings(HID_Subsystem dc) {

        // get an xbox controller for the operator, or null
        CommandXboxController opr = (dc.Operator() instanceof CommandXboxController)
                ? (CommandXboxController) dc.Operator()
                : null;

        // for end effector
        opr.rightBumper().whileTrue(new EndEffectorPercent(-.3, "rightBumper")); // reverse
        opr.rightTrigger().whileTrue(new EndEffectorPercent(.5, "rightTrigger")); // p

        opr.x().onTrue(new setWristPos(WristFLA.PICKUP_POSITION, "x"));
        opr.y().onTrue(new setWristPos(WristFLA.MID_POSITION, "y"));
        opr.a().onTrue(new setWristPos(WristFLA.DROP_POSITION, "a"));

    }
}
