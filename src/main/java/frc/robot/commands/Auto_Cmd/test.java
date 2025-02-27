package frc.robot.commands.Auto_Cmd;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveDrive;
import frc.robot.LimelightHelpers;

public class test extends Command {
    private final SwerveSubsystem swerve;

    public test(SwerveSubsystem swerve, SwerveDrive swerveDrive){
        this.swerve = swerve;
        addRequirements(this.swerve);
    }

    public void excute(){
        // LimelightHelpers.getOrientation(swerve.getSwerveDrive());
        PoseEstimate Bot = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("");
        Pose2d BotPose = Bot.pose;

        // swerve.resetOdometry(BotPose);
        swerve.resetOdometry(new Pose2d(1.901, 4.031, Rotation2d.fromDegrees(0)));
        // swerve.driveToPose(new Pose2d(3.715, 5.351, Rotation2d.fromDegrees(0)))  //19
        swerve.driveToPose(new Pose2d(2.901, 4.031, Rotation2d.fromDegrees(0)))  //18
              .andThen( () -> swerve.getSwerveDrive().lockPose());
    }
}
