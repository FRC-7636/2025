package frc.robot.commands.Auto_Cmd;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class AutoDriveOneMeter extends Command{
    private final SwerveSubsystem swerve;

    PathPlannerPath path = null;

    public AutoDriveOneMeter(SwerveSubsystem swerve) {
        this.swerve = swerve;
        addRequirements(this.swerve);
        try {
            path = PathPlannerPath.fromPathFile("one_meter");
            swerve.getSwerveDrive().resetOdometry(new Pose2d(7.5, 7, Rotation2d.fromDegrees(0)));
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
        swerve.getSwerveDrive().resetOdometry(new Pose2d(7.5, 7, Rotation2d.fromDegrees(0)));
        Commands.runOnce( () -> swerve.driveToPose(new Pose2d(5.5, 7, Rotation2d.fromDegrees(0))), swerve);
        // swerve.driveToPose(new Pose2d(1, 0, Rotation2d.fromDegrees(0)));
        // swerve.driveToPose(new Pose2d(0, 1, Rotation2d.fromDegrees(0)));

        // Commands.runOnce( () -> AutoBuilder.followPath(path));
    }

    @Override
    public void end(boolean interrupted) {
    }
}