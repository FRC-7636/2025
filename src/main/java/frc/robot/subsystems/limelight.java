package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Rotation;

import java.security.PublicKey;
import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.LimelightHelpers;

public class limelight extends SubsystemBase{
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    public Field2d field2d;
    public static AprilTagFieldLayout aprilTagFieldLayout;

    public Field2d field2ddd;
    public static AprilTagFieldLayout aprilTagFieldLayout222;

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

    public static Pose2d getAprilTagPose(){
        if (TagID != -1){
            // Optional<Pose3d> aprilTagPose3d = aprilTagFieldLayout.getTagPose(TagID);
            tag = true;
            // return aprilTagPose3d.get().toPose2d().transformBy(robotOffset);
            // return aprilTagPose3d.get().toPose2d();
            return aprilTagFieldLayout.getTagPose(TagID).get().toPose2d();
        }
        else{
            tag = false;
            // throw new RuntimeException("Cannot get AprilTag " + aprilTag + " from field " + aprilTagFieldLayout.toString());
            return new Pose2d();
        }
    }

        public Transform2d getdeltaPose(){
            Rotation2d tagRotation2d = new Rotation2d(Math.toRadians(90));
            Pose2d RobotPose = LimelightHelpers.getBotPose2d_wpiBlue("");
            Pose2d tagPose = getAprilTagPose();
            return RobotPose.minus(tagPose);
        }

    @Override
    public void periodic(){
        if(TagID != 1){
            getTag();
            getRobotPose();
            robotToTarget();
            deltaRobotHeadingDeg();
            getAprilTagPose();
        }

        field2d.setRobotPose(getRobotPose());
        SmartDashboard.putData("field2d", field2d);

        SmartDashboard.putNumber("RY", LimelightHelpers.getTargetPose3d_CameraSpace("").getRotation().getY()*57.3);
        SmartDashboard.putNumber("Tag ID", TagID);
        SmartDashboard.putBoolean("getTag", tag);
        // SmartDashboard.putNumber("Robot_Heading_Degree", deltaRobotHeadingDeg());
   }
}
