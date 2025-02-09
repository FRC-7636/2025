package frc.robot.commands.Auto;

import java.io.IOException;

import org.json.simple.parser.ParseException;
import org.opencv.core.Mat;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.limelight;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveDrive;

public class AutoDriveToBarge extends SequentialCommandGroup {
    private PIDController driveCtrl = new PIDController(Constants.AutoConstants.AutoDrivePIDF.P, 
                                                        Constants.AutoConstants.AutoDrivePIDF.I, 
                                                        Constants.AutoConstants.AutoDrivePIDF.D);
    private PIDController turnCtrl = new PIDController(Constants.AutoConstants.AutoTurnPIDF.P, 
                                                       Constants.AutoConstants.AutoTurnPIDF.I, 
                                                       Constants.AutoConstants.AutoTurnPIDF.D);

    public AutoDriveToBarge(SwerveSubsystem swerve, limelight limelight){
        // Pose2d BotPose = LimelightHelpers.getBotPose2d_wpiBlue("");

        // Rotation2d tagRotation2d = new Rotation2d(Math.toRadians(90));
        // Pose2d RobotPose = swerve.getPose();
        // Pose2d tagPose = new Pose2d(16.087, 1.168, tagRotation2d);
        // Transform2d deltaTransform2d = RobotPose.minus(tagPose);
        // Translation2d targetTranslation2d = deltaTransform2d.getTranslation();
        // Rotation2d targetRotation2d = deltaTransform2d.getRotation();
        // double deltaDeg = targetRotation2d.getDegrees();

        // boolean auto = false;

        // PathPlannerPath path = null;
        // try {
        //     path = PathPlannerPath.fromPathFile("New Path");
        //     swerve.getSwerveDrive().resetOdometry(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
            
        // } catch (FileVersionException e) {
        //     // TODO Auto-generated catch block
        //     e.printStackTrace();
        // } catch (IOException e) {
        //     // TODO Auto-generated catch block
        //     e.printStackTrace();

        // } catch (ParseException e) {
        //     // TODO Auto-generated catch block
        //     e.printStackTrace();
        // }
        
        // if(RobotPose.getX() != 0){
            addRequirements(swerve);
            // addCommands(Commands.runOnce(() -> swerve.resetOdometry(new Pose2d(0, 0, Rotation2d.fromDegrees(0)))));
            // addCommands(AutoBuilder.followPath(path));

            // addCommands(Commands.runOnce(() -> swerve.resetOdometry(new Pose2d(1.642, 3.348, Rotation2d.fromDegrees(0)))));
            addCommands( swerve.driveToPose(new Pose2d(2.901, 4.301, Rotation2d.fromDegrees(0))));

        // }
    }
}
