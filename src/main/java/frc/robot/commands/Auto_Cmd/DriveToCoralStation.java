package frc.robot.commands.Auto_Cmd;

import java.io.IOException;
import org.json.simple.parser.ParseException;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.limelight;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class DriveToCoralStation extends SequentialCommandGroup{
    private Pose2d avgPose;
    
    public DriveToCoralStation(SwerveSubsystem swerve, limelight limelight){
        boolean auto = false;
        PathPlannerPath path = null;
        try {
            path = PathPlannerPath.fromPathFile("DriveToReef18_2");

            // swerve.getSwerveDrive().resetOdometry(new Pose2d(0.901, 4.031, Rotation2d.fromDegrees(0)));
            swerve.getSwerveDrive().resetOdometry(LimelightHelpers.getBotPose2d_wpiBlue("limelight-two"));

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
        Pose2d LLrobot_1 = limelight.getRobotPose().toPose2d();
        Pose2d LLrobot_2 = limelight.getRobotPose_two().toPose2d();

        addRequirements(swerve);
        addCommands(Commands.run( () -> { 
                                            if(LimelightHelpers.getFiducialID("") == -1){
                                                swerve.getSwerveDrive().resetOdometry(LLrobot_2); 
                                            }
                                            else if(LimelightHelpers.getFiducialID("limelight-two") == -1){
                                                swerve.getSwerveDrive().resetOdometry(LLrobot_1); 
                                            }
                                            else if (LimelightHelpers.getFiducialID("limelight-two") != -1 && LimelightHelpers.getFiducialID("limelight-two") != -1){
                                                avgPose = new Pose2d( (LLrobot_1.getX() + LLrobot_2.getX() ) / 2,
                                                                      (LLrobot_1.getY() + LLrobot_2.getY() ) / 2, 
                                                                       LLrobot_1.getRotation().plus(LLrobot_2.getRotation()).div(2)
                                                                    );
                                                swerve.getSwerveDrive().resetOdometry(avgPose);
                                            }
                                            else {
                                                swerve.getSwerveDrive().resetOdometry(swerve.getPose());
                                            }
                                        }
                                )
                    );   

        addCommands(new WaitCommand(0.5));
        addCommands(Commands.runOnce( () -> swerve.driveToPose(new Pose2d(1.258, 7.126, Rotation2d.fromDegrees(36))), swerve));
    }

}
