package frc.robot.commands.Auto_Cmd;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.limelight;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.swervedrive.Vision;

public class DriveToReef18 extends SequentialCommandGroup{
    private Pose2d avgPose;

    
    public DriveToReef18(SwerveSubsystem swerve, limelight limelight, Vision vision){
        // boolean auto = false;
        PathPlannerPath path = null;
        try {
            path = PathPlannerPath.fromPathFile("DriveToReef18_1");

            // swerve.getSwerveDrive().resetOdometry(new Pose2d(0.901, 4.031, Rotation2d.fromDegrees(0)));

        } catch (FileVersionException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        } catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();

        } catch (ParseException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }

        // Pose2d LLrobot_1 = LimelightHelpers.getBotPose2d_wpiBlue("");
        // Pose2d LLrobot_2 = LimelightHelpers.getBotPose2d_wpiBlue("limelight-two");

        // Get Robot's Pose from Limelight
        // if(LimelightHelpers.getFiducialID("") == -1){
        //     avgPose = LLrobot_2;
        //     swerve.getSwerveDrive().resetOdometry(avgPose); 
        // }
        // else if(LimelightHelpers.getFiducialID("limelight-two") == -1){
        //     avgPose = LLrobot_2;
        //     swerve.getSwerveDrive().resetOdometry(avgPose); 
        // }
        // else if (LimelightHelpers.getFiducialID("limelight-two") != -1 && LimelightHelpers.getFiducialID("limelight-two") != -1){
        //     avgPose = new Pose2d( (LLrobot_1.getX() + LLrobot_2.getX() ) / 2,
        //                           (LLrobot_1.getY() + LLrobot_2.getY() ) / 2, 
        //                            LLrobot_1.getRotation().plus(LLrobot_2.getRotation()).div(2)
        //                         );
        //     swerve.getSwerveDrive().resetOdometry(avgPose);
        // }
        // else {
        //     avgPose = swerve.getPose();
        //     swerve.getSwerveDrive().resetOdometry(avgPose);
        // }

        addRequirements(swerve, limelight, vision);
        // addCommands(Commands.runOnce( () -> limelight.getLLPose(), limelight));

        // addCommands(Commands.run( () -> { 
        //                                 if(LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("")  != null || LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-two")  != null){
        //                                     swerve.getSwerveDrive().resetOdometry(limelight.getLLPose());
        //                                 }
        //                                 else {
        //                                     // avgPose = swerve.getPose();
        //                                     swerve.getSwerveDrive().resetOdometry(swerve.getPose());
        //                                 }
        //                                 }
        //                         )
        //             );
        // addCommands(Commands.runOnce( () -> swerve.getSwerveDrive().resetOdometry(limelight.getLLPose()), swerve));
                                        
        // addCommands(new WaitCommand(5));
        // addCommands(Commands.runOnce( () -> limelight.getLLPose(), limelight));
        addCommands(Commands.runOnce( () -> swerve.getSwerveDrive().resetOdometry(LimelightHelpers.getBotPose2d_wpiBlue("")), swerve));
        // addCommands(Commands.runOnce( () -> swerve.driveToPose(new Pose2d(2.901, 4.031, Rotation2d.fromDegrees(0))), swerve));

        // addCommands(AutoBuilder.followPath(path));
        addCommands(Commands.runOnce( () -> swerve.driveToPose(new Pose2d(2.901, 4.031, Rotation2d.fromDegrees(0))), swerve));
        // addCommands(Commands.runOnce( () -> swerve.driveToPose(new Pose2d(vision.getTagPose().getX(),
        //                                                                   vision.getTagPose().getY(),
        //                                                                   Rotation2d.fromDegrees(vision.getTagPose().getRotation().getDegrees())))));
        // addCommands(Commands.runOnce( () -> {
        //                                         if(LimelightHelpers.getFiducialID("") == -1 || LimelightHelpers.getFiducialID("limelight-two") == -1){
        //                                             while (Math.abs(limelight.getLLPose().getRotation().getDegrees()) > 5 || swerve.getPose().getRotation().getDegrees() > 5) {
        //                                                 swerve.driveToPose(new Pose2d(2.901, 4.031, Rotation2d.fromDegrees(0)));
        //                                             }
        //                                         }
        //                                     }));
    }
}
