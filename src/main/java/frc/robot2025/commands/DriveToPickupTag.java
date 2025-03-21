package frc.robot2025.commands;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib2202.builder.RobotContainer;
import frc.lib2202.command.pathing.MoveToPose;
import frc.lib2202.subsystem.OdometryInterface;
import frc.robot2025.Constants.TheField;
import frc.robot2025.subsystems.Limelight;

public class DriveToPickupTag extends Command{
    static double LeftOffset =  -0.04;  //[m]
    static double RightOffset = -0.39;  //[m]
    static double BackupOffset = 0.55; //[m]

    static PathConstraints constraints = new PathConstraints(2.5, 1.75, Math.PI, Math.PI / 2.0);

    final Limelight LL;
    final String LLName;
    final OdometryInterface odo;
    final String odoName = "vision_odo";   //todo make an arg
    final int tagIdx;

    // command vars set at init time
    boolean done;
    Pose2d target;
    Command moveComand;
    public DriveToPickupTag(String side){
        odo = RobotContainer.getSubsystemOrNull(odoName);
        LL = RobotContainer.getObjectOrNull("limelight");
        LLName = (LL != null) ? LL.getName() : "no-ll-found";  //name if we need to use LLHelpers directly

        // pick a direction to go, left , right in TheField
        tagIdx = side.toLowerCase().startsWith("l") ? 0 : 1;        
    }

    @Override
    public void initialize() {
        done = true;
        //protect from missng required ss
        if (LL == null) return;
        if (odo == null) return;
        moveComand = null;       

        Alliance alliance;
        // get our alliance
        if (DriverStation.getAlliance().isPresent()) {
            alliance = DriverStation.getAlliance().get();            
        }
        else return; // no alliance, bail

         // set LL targets to our reef only
        int tagId = (alliance == Alliance.Blue) ? TheField.PickupIdsBlue[tagIdx] : TheField.PickupIdsRed[tagIdx];
        target = TheField.fieldLayout.getTagPose(tagId).get().toPose2d();

        // TODO - apply backup & L/R offset to target

        moveComand = new MoveToPose(odoName, constraints, target);
        moveComand.schedule();           
        //made it this far, start looking for reef tages in execute()
        done = false;        
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
           if (moveComand != null && moveComand.isScheduled()) {
             moveComand.cancel();
           }
        }
    }

    @Override
    public boolean isFinished() {
        if (moveComand != null) {
            done = moveComand.isFinished();
        }
        return done;
    }

}
