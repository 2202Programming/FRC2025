package frc.robot2025;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib2202.command.swerve.RotateUntilSeeTags;
import frc.robot2025.Constants.Tag_Pose;

/*
 * Place commands named in PathPlaner autos here.
 */
public class RegisteredCommands {

    public static SendableChooser<Command> RegisterCommands() {
        SendableChooser<Command> autoChooser;



        
        NamedCommands.registerCommand("RotateTo", 
                new RotateUntilSeeTags(Tag_Pose.ID4, Tag_Pose.ID7));


        autoChooser = AutoBuilder.buildAutoChooser();
        // select our auto
        SmartDashboard.putData("Auto Chooser", autoChooser);
        return autoChooser;
    }
}