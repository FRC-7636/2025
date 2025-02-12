package frc.robot.commands.Auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.limelight;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.swervedrive.Vision;

public class Reef extends SequentialCommandGroup{

    PIDController controller = new PIDController(1, 0, 0);
    boolean stop;
    
    public Reef(SwerveSubsystem swerve, limelight limelight, Vision vision){
        
        // Pose2d BotPose = swerve.getPose();
        // Pose2d TagPose = limelight.getAprilTagPose();

        // Transform2d TransPose = BotPose.minus(TagPose);


        // addRequirements(swerve);
        // addCommands(Commands.runOnce(() -> controller.enableContinuousInput(-Math.PI, Math.PI), swerve));
        // // addCommands(Commands.runOnce(() -> swerve.getReefYaw(), swerve));
        // addCommands(Commands.runOnce(() -> {swerve.aimTarget();}, swerve));
        // stop = false;
        // System.out.println("stop");
        
        // addCommands(Commands.runOnce(() -> vision.getPoseFromTag(), vision));

        // addCommands(Commands.runOnce(() -> swerve.driveToPose(new Pose2d(12, 6, new Rotation2d(0))), swerve));
        // addCommands(Commands.runOnce(null, null));

    }
    
}
