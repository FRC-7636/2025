package frc.robot.commands.Auto;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.limelight;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class AutoDriveToBarge extends SequentialCommandGroup {

    public AutoDriveToBarge(SwerveSubsystem swerve, limelight limelight){
        // Pose2d BotPose = LimelightHelpers.getBotPose2d_wpiBlue("");

        // Rotation2d tagRotation2d = new Rotation2d(Math.toRadians(90));
        // Pose2d RobotPose = swerve.getPose();
        // Pose2d tagPose = new Pose2d(16.087, 1.168, tagRotation2d);
        // Transform2d deltaTransform2d = RobotPose.minus(tagPose);
        // Translation2d targetTranslation2d = deltaTransform2d.getTranslation();
        // Rotation2d targetRotation2d = deltaTransform2d.getRotation();
        // double deltaDeg = targetRotation2d.getDegrees();

        boolean auto = false;

        PathPlannerPath path = null;
        try {
            // path = PathPlannerPath.fromPathFile("one_meter");
            path = PathPlannerPath.fromPathFile("New New Path");

            // swerve.getSwerveDrive().resetOdometry(new Pose2d(0.901, 4.031, Rotation2d.fromDegrees(0)));
            swerve.getSwerveDrive().resetOdometry(LimelightHelpers.getBotPose2d_wpiBlue("limelight-two"));

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
        Pose2d LLrobot = LimelightHelpers.getBotPose2d_wpiBlue("limelight-two");
        // Pose2d robot = limelight.getRobotPose_two().toPose2d();
        // if(RobotPose.getX() != 0){
            addRequirements(swerve);
            // addCommands(Commands.runOnce(() -> swerve.resetOdometry(new Pose2d(1.642, 4.031, Rotation2d.fromDegrees(0)))));
            // addCommands(Commands.runOnce(() -> swerve.resetOdometry(swerve.getPose())));
            // swerve.getSwerveDrive().resetOdometry(new Pose2d(0.901, 4.031, Rotation2d.fromDegrees(0)));
            addCommands(Commands.run( () -> { 
                                                if(LimelightHelpers.getFiducialID("limelight-two") != -1){
                                                    swerve.getSwerveDrive().resetOdometry(LLrobot); 
                                                }
                                            }
                                    )
                        );
            addCommands(new WaitCommand(0.5));
            addCommands(Commands.runOnce( () -> swerve.driveToPose(new Pose2d(2.901, 4.031, Rotation2d.fromDegrees(0))), swerve));
            // addCommands(AutoBuilder.followPath(path));

            // addCommands(Commands.runOnce( () -> swerve.resetOdometry(new Pose2d(1.642, 4.031, Rotation2d.fromDegrees(0)))));
            // addCommands( swerve.driveToPose(new Pose2d(3.051, 4.251, Rotation2d.fromDegrees(0))));
        // }
    }
}
