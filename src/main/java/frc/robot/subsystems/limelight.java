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
    NetworkTable table2 = NetworkTableInstance.getDefault().getTable("limelight2");

    public Field2d field2d;
    public static AprilTagFieldLayout aprilTagFieldLayout;

    public static int TagID = (int) LimelightHelpers.getFiducialID("");
    public static boolean tag = false;
    public static Transform2d robotOffset;

    public limelight(){
        field2d = new Field2d();
        aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
    }

    public void getTag(){
        TagID = (int) LimelightHelpers.getFiducialID("");
        if(TagID == -1){
            tag = false;
        }
        else{
            tag = true;
        }
    }

    public Pose2d getRobotPose(){
        return LimelightHelpers.getBotPose2d_wpiBlue("");
    }

    public Pose3d robotToTarget(){
        return LimelightHelpers.getBotPose3d_TargetSpace("");
    }

    public double deltaRobotHeadingDeg(){
        return LimelightHelpers.getBotPose2d("").getRotation().getDegrees();
    }    

    @Override
    public void periodic(){
        if(TagID != 1){
            getTag();
            getRobotPose();
            robotToTarget();
            deltaRobotHeadingDeg();
        }
        field2d.setRobotPose(getRobotPose());
        SmartDashboard.putData("field2d", field2d);

        SmartDashboard.putNumber("RY", LimelightHelpers.getTargetPose3d_CameraSpace("").getRotation().getY()*57.3);
        SmartDashboard.putNumber("Tag ID", TagID);
        SmartDashboard.putBoolean("getTag", tag);

   }
}
