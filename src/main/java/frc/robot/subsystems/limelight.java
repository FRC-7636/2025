package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class limelight extends SubsystemBase{
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTable table2 = NetworkTableInstance.getDefault().getTable("limelight-two");

    public Field2d LL_Pose;
    public static AprilTagFieldLayout aprilTagFieldLayout;

    public int TagID, TagID2;
    public static boolean tag = false;
    public static Transform2d robotOffset;

    public limelight(){
        LL_Pose = new Field2d();
        aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
    }

    public boolean getTag(){
        TagID = (int) LimelightHelpers.getFiducialID("");
        TagID2 = (int) LimelightHelpers.getFiducialID("two");

        if(TagID == -1 && TagID2 == -1){
            return tag = false;
        }
        else if(TagID != -1 || TagID2 != -1){
            return tag = true;
        }
        return false;
    }

    public Pose2d getRobotPose(){
        return LimelightHelpers.getBotPose2d_wpiBlue("");
    }

    public Pose2d getRobotPose_two(){
        return LimelightHelpers.getBotPose2d("two");
    }

    public Pose3d robotToTarget(){
        return LimelightHelpers.getBotPose3d_TargetSpace("");
    }

    public double deltaRobotHeadingDeg(){
        return LimelightHelpers.getBotPose2d("").getRotation().getDegrees();
    }    

    @Override
    public void periodic(){
        if(tag = true){
            getTag();
            getRobotPose();
            getRobotPose_two();
            robotToTarget();
            deltaRobotHeadingDeg();
        }

        Pose2d avgPos = new Pose2d(
                                  (getRobotPose().getX() + getRobotPose_two().getX()) / 2,
                                  (getRobotPose().getY() + getRobotPose_two().getY()) / 2,
                                   getRobotPose().getRotation().plus(getRobotPose_two().getRotation()).div(2)
        );

        LL_Pose.setRobotPose(avgPos);
        SmartDashboard.putData("LL_Pose", LL_Pose);
        // SmartDashboard.putNumber("RY", LimelightHelpers.getTargetPose3d_CameraSpace("").getRotation().getY() * 57.3);
        SmartDashboard.putBoolean("getTag", tag);
   }
}
