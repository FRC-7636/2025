package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Rotation;

import java.util.Optional;

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
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.LimelightHelpers;

public class limelight extends SubsystemBase{
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    public Field2d field2d;
    public static AprilTagFieldLayout aprilTagFieldLayout;

    public Field2d field2ddd;
    public static AprilTagFieldLayout aprilTagFieldLayout222;

    public static int aprilTag;
    public static boolean tag;

    public limelight(){
        field2d = new Field2d();
        aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
    }

    public Pose3d getRobotPose(){
        return LimelightHelpers.getBotPose3d_wpiBlue("");
    }

    public Pose3d robotToTarget(){
        return LimelightHelpers.getBotPose3d_TargetSpace("");
    }

    public double deltaRobotHeadingDeg(){
        return LimelightHelpers.getBotPose2d("").getRotation().getDegrees();
    }    

    public static Pose2d getAprilTagPose(Transform2d robotOffset){
        if (aprilTag != -1 ){
            aprilTag = (int) LimelightHelpers.getFiducialID("");
        }
        Optional<Pose3d> aprilTagPose3d = aprilTagFieldLayout.getTagPose(aprilTag);

        if (aprilTagPose3d.isPresent()){
            tag = true;
            System.out.println(aprilTagPose3d);
            return aprilTagPose3d.get().toPose2d().transformBy(robotOffset);
        }
        else{
            //   throw new RuntimeException("Cannot get AprilTag " + aprilTag + " from field " + aprilTagFieldLayout.toString());
            tag = false;
            return null;
      }
    }

    @Override
    public void periodic(){
        getRobotPose();
        robotToTarget();
        deltaRobotHeadingDeg();
        SmartDashboard.putNumber("RY", LimelightHelpers.getTargetPose3d_CameraSpace("").getRotation().getY()*57.3);
        field2d.setRobotPose(getRobotPose().toPose2d());
        SmartDashboard.putData("field2d", field2d);
        // SmartDashboard.putNumber("Robot_Heading_Degree", deltaRobotHeadingDeg());
   }
}
