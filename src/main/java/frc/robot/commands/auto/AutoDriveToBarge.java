package frc.robot.commands.auto;

import org.opencv.core.Mat;

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

public class AutoDriveToBarge extends SequentialCommandGroup {
    private PIDController driveCtrl = new PIDController(Constants.AutoDrivePIDF.P, Constants.AutoDrivePIDF.I, Constants.AutoDrivePIDF.D);
    private PIDController turnCtrl = new PIDController(Constants.AutoTurnPIDF.P, Constants.AutoTurnPIDF.I, Constants.AutoTurnPIDF.D);

    public AutoDriveToBarge(SwerveSubsystem swerve, limelight limelight){
        Pose2d BotPose = LimelightHelpers.getBotPose2d_wpiBlue("");

        Rotation2d tagRotation2d = new Rotation2d(Math.toRadians(90));
        Pose2d RobotPose = swerve.getPose();
        Pose2d tagPose = new Pose2d(16.087, 1.168, tagRotation2d);
        Transform2d deltaTransform2d = RobotPose.minus(tagPose);
        Translation2d targetTranslation2d = deltaTransform2d.getTranslation();
        Rotation2d targetRotation2d = deltaTransform2d.getRotation();
        double deltaDeg = targetRotation2d.getDegrees();

        boolean auto = false;
        
        if(RobotPose.getX() != 0){
            addCommands(Commands.runOnce(() -> swerve.resetOdometry(LimelightHelpers.getBotPose2d_wpiBlue("")), swerve));
            addCommands(swerve.getAutonomousCommand("New Path"));
            addCommands(Commands.runOnce(() -> swerve.resetOdometry(LimelightHelpers.getBotPose2d_wpiBlue("")), swerve));
            addCommands(swerve.getAutonomousCommand("New Path"));
        }
    }
}
