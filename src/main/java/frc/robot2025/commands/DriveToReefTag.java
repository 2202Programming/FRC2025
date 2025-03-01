package frc.robot2025.commands;
import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib2202.command.pathing.MoveToPose;
import frc.robot2025.Constants.TheField;


public class DriveToReefTag extends Command {

    //Robot left/right offsets for aligning with reef - TODO fix L/R values
    static double LeftOffset = -0.3;  //[m]
    static double RightOffset = 0.2;  //[m]
    static double BackupOffset = 0.4; //[m]
    static Rotation2d LLRot = Rotation2d.kCW_90deg; // ll on side, need to add this for final pose

    static Map<Integer, Pose2d> blueReefLeft = new HashMap<Integer, Pose2d>();
    static Map<Integer, Pose2d> blueReefRight = new HashMap<Integer, Pose2d>();
    static Map<Integer, Pose2d> redReefLeft = new HashMap<Integer, Pose2d>();
    static Map<Integer, Pose2d> redReefRight = new HashMap<Integer, Pose2d>();

    @SuppressWarnings("unused")
    static void buildPositions(Map<Integer, Pose2d> map, int[] tags, double lr_offset) {
        // loop over given tags and build the 2d targets
        double x, y, rot;
        for (int tagId : tags) {
            Pose3d tagPose = TheField.fieldLayout.getTagPose(tagId).get();
            Pose2d foo = tagPose.toPose2d();  //test
            var matrix = tagPose.toMatrix();
            x = tagPose.getX();
            y = tagPose.getY();
            rot = tagPose.getRotation().getAngle();
            var rotdeg = rot*57.3; //debug assist
            var rot2d = new Rotation2d(rot);

            // Backup robot along tag face
            double dx = rot2d.getCos()*BackupOffset;
            double dy = rot2d.getSin()*BackupOffset;

            // adjust L/R
            double lr_dx = rot2d.getSin()*lr_offset;
            double lr_dy = rot2d.getCos()*lr_offset;

            Pose2d targetPose = new Pose2d(x +dx + lr_dx,
                                           y + dy + lr_dy, rot2d.plus(LLRot));
            
            map.put(tagId, targetPose);            
        }
    }

    // setup our targets
    static {
        buildPositions(blueReefLeft, TheField.ReefIdsBlue, LeftOffset);
        buildPositions(blueReefRight, TheField.ReefIdsBlue, RightOffset);
        buildPositions(redReefLeft, TheField.ReefIdsRed, LeftOffset);
        buildPositions(redReefRight, TheField.ReefIdsRed, RightOffset);
    }
    
    final boolean leftSide;  //side of reef to deliver to
    final Map<Integer, Pose2d> redPoses;
    final Map<Integer, Pose2d> bluePoses;

    // command vars set at init time
    boolean done;
    Command moveComand;
    Map<Integer, Pose2d> alliancePoses;

    public DriveToReefTag(String reefSide) {
        // pick a direction to go
        leftSide = reefSide.toLowerCase().startsWith("l");

        // setup red/blue for this commands side
        redPoses = leftSide ? redReefLeft : redReefRight;
        bluePoses = leftSide ? blueReefLeft : blueReefRight;
    }
    
    @Override
    public void initialize() {
        done = false;  //true when we found a reef tag or gave up
        moveComand = null;       
        done = true;   

        Alliance alliance;
        // get our alliance
        if (DriverStation.getAlliance().isPresent()) {
            alliance = DriverStation.getAlliance().get();            
        }
        else return;
        alliancePoses = (alliance == Alliance.Blue) ? bluePoses : redPoses;
        
        // set LL targets to our side
        @SuppressWarnings("unused")  //wip
        var tags = alliancePoses.keySet();
        //LL call 

    }

    @Override
    public void execute() { 
        // Look for our tags and create a moveTo if we find 
        int foundTag = 0;

        // read LL for tag

        if (foundTag > 0 ) { //also check it is good...
            Pose2d targetPose = alliancePoses.get(foundTag);
            // build path to target
            moveComand = new MoveToPose(targetPose);
            moveComand.schedule();
            done = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
           if (moveComand != null) moveComand.cancel();
        }
    }

    @Override
    public boolean isFinished() {
        return done;
    }

}
