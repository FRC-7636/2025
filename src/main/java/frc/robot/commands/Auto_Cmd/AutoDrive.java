package frc.robot.commands.Auto_Cmd;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class AutoDrive extends Command{
    private final SwerveSubsystem swerve;

    PathPlannerPath path = null;

    public AutoDrive(SwerveSubsystem swerve) {
        this.swerve = swerve;
        addRequirements(this.swerve);
        try {
            path = PathPlannerPath.fromPathFile("DriveToReef18_2");
            swerve.getSwerveDrive().resetOdometry(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
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
    @Override
    public void execute(){
        swerve.resetOdometry(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
        AutoBuilder.followPath(path);
    }

    @Override
    public void end(boolean interrupted) {
    }
}