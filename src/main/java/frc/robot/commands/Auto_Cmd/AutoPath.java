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
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveDrive;

public class AutoPath extends Command{
    private final SwerveSubsystem swerve;

    PathPlannerPath path = null;

    public AutoPath(SwerveSubsystem swerve, SwerveDrive swerveDrive){
        this.swerve = swerve;
        addRequirements(this.swerve);
        try {
                // path = PathPlannerPath.fromPathFile("New New New Path");
                path = PathPlannerPath.fromPathFile("Left one");

                // swerve.getSwerveDrive().resetOdometry(new Pose2d(2.175, 5.351, Rotation2d.fromDegrees(0)));
                // swerve.getSwerveDrive().resetOdometry(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
                swerve.getSwerveDrive().resetOdometry(new Pose2d(3.560, 6.117, Rotation2d.fromDegrees(0)));
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
    }

    public void excute(){
        // swerve.resetOdometry(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
        swerve.resetOdometry(new Pose2d(3.560, 6.117, Rotation2d.fromDegrees(0)));

        AutoBuilder.followPath(path)
                                    .andThen(Commands.runOnce(() -> swerve.getSwerveDrive().lockPose()));
        swerve.resetOdometry(new Pose2d(2.560, 6.117, Rotation2d.fromDegrees(0)));
        swerve.resetOdometry(new Pose2d(2.560, 6.117, Rotation2d.fromDegrees(180)));
    }
}
