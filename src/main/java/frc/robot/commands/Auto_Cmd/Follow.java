package frc.robot.commands.Auto_Cmd;

import java.io.IOException;

import org.ejml.dense.row.CommonOps_MT_CDRM;
import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class Follow extends SequentialCommandGroup{
    PathPlannerPath path = null;
    
    public Follow(SwerveSubsystem swerve){
        swerve.resetOdometry(new Pose2d(7.5, 7, Rotation2d.fromDegrees(180)));

        try{
            path = PathPlannerPath.fromPathFile("Blue_Left_1");
        }
        catch (FileVersionException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        } catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();

        } catch (ParseException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }

        // addCommands(Commands.runOnce(() -> swerve.resetOdometry(limelight.getPose2d()), swerve));
        addCommands(Commands.runOnce( () -> swerve.resetOdometry(new Pose2d(7.500, 7.000, Rotation2d.fromDegrees(180))), swerve));
        
        var alliance = DriverStation.getAlliance();
        if (alliance.get() == DriverStation.Alliance.Blue){
            addCommands(AutoBuilder.followPath(path));
        }
        else{
            System.out.println("Wrong Auto");
        }
        // addCommands(Commands.runOnce( () -> swerve.lock(), swerve));
    }
}

