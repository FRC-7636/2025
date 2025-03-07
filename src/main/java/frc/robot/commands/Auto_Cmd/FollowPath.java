package frc.robot.commands.Auto_Cmd;

import java.io.IOException;
import java.security.PublicKey;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.limelight;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class FollowPath extends Command{
//     private final SwerveSubsystem swerve;
//     private final Elevator elevator;
//     private final limelight limelight;

//     PathPlannerPath path = null;
    
//     public FollowPath(SwerveSubsystem swerve, Elevator elevator, limelight limelight){
//         this.swerve = swerve;
//         this.elevator = elevator;
//         this.limelight = limelight;
//         addRequirements(this.swerve, this.elevator, this.limelight);
        
//         try {
//             path = PathPlannerPath.fromPathFile("Down_one_meter");

//         } catch (FileVersionException e) {
//             // TODO Auto-generated catch block
//             e.printStackTrace();
//         } catch (IOException e) {
//             // TODO Auto-generated catch block
//             e.printStackTrace();

//         } catch (ParseException e) {
//             // TODO Auto-generated catch block
//             e.printStackTrace();
//         }
//     }

//         @Override
//         public void execute(){
//             Commands.runOnce( () -> swerve.resetOdometry(new Pose2d(1.560, 6.117, Rotation2d.fromDegrees(0))));
//             Commands.runOnce( () -> swerve.resetOdometry(limelight.getLLPose()));
//             AutoBuilder.followPath(path);
//             Commands.runOnce( () -> swerve.resetOdometry(limelight.getLLPose()));
//             Commands.runOnce( () -> {
//                                     if(LimelightHelpers.getFiducialID("") == -1 || LimelightHelpers.getFiducialID("limelight-two") == -1){
//                                         while (Math.abs(limelight.getLLPose().getRotation().getDegrees()) > 5 || swerve.getPose().getRotation().getDegrees() > 5) {
//                                             swerve.driveToPose(new Pose2d(2.901, 4.031, Rotation2d.fromDegrees(0)));
//                                         }
//                                     }
//                                 });
//             new WaitCommand(5);
//             Commands.runOnce( () -> elevator.ELE_RL2(),  elevator);
//         }
}
