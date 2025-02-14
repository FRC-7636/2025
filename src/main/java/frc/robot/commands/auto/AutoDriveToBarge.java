package frc.robot.commands.Auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class AutoDriveToBarge extends SequentialCommandGroup {

    public AutoDriveToBarge(SwerveSubsystem swerve){
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
            addCommands( swerve.driveToPose(new Pose2d(3.051, 4.251, Rotation2d.fromDegrees(0))));
        // }
    }
}
