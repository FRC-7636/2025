package frc.robot.commands.auto;

import javax.xml.crypto.KeySelector.Purpose;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.*;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.LimelightHelpers;
import frc.robot.Constants;

public class AutoDrive extends Command{
    private final SwerveSubsystem swerve;
    private final limelight limelight;
    private boolean stop;
    private boolean stop2;

    private PIDController driveCtrl = new PIDController(Constants.AutoDrivePIDF.P, Constants.AutoDrivePIDF.I, Constants.AutoDrivePIDF.D);
    private PIDController turnCtrl = new PIDController(Constants.AutoTurnPIDF.P, Constants.AutoTurnPIDF.I, Constants.AutoTurnPIDF.D);
    
    public AutoDrive(SwerveSubsystem swerve, limelight limelight) {
        this.swerve = swerve;
        this.limelight = limelight;
        addRequirements(this.swerve, this.limelight);
    }

    @Override
    public void execute(){
        Pose2d LLPose = LimelightHelpers.getBotPose2d("");
        
        if (LLPose.getX() != 0) {
            swerve.resetOdometry(LLPose);
            stop = false;
        }
        else {
            stop = true; 
        }
        
        Rotation2d tagRotation2d = new Rotation2d(Math.toRadians(90));
        Pose2d RobotPose = swerve.getPose();
        Pose2d tagPose = new Pose2d(16.087, 1.168, tagRotation2d);
        Transform2d deltaTransform2d = RobotPose.minus(tagPose);
        Translation2d targetTranslation2d = deltaTransform2d.getTranslation();
        Rotation2d targetRotation2d = deltaTransform2d.getRotation();
        double deltaDeg = targetRotation2d.getDegrees();
        
        if ((Math.abs(deltaTransform2d.getX()) < 3) && (Math.abs(deltaTransform2d.getY()) < 3) && (Math.abs(deltaDeg) < 3)){
            swerve.drive(targetTranslation2d, turnCtrl.calculate(deltaDeg), false);
        }
        else {
            swerve.drive(targetTranslation2d, 0, true);
            // swerve.drive(targetTranslation2d, turnCtrl.calculate(deltaDeg), false);
        }
    }

    // @Override
    // public boolean isFinished(){
    //     return stop;
    // }

    // @Override
    // public void end(boolean interrupted) {
    // }
    
}