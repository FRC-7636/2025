package frc.robot.commands.auto;

import java.util.concurrent.TransferQueue;

import javax.xml.crypto.KeySelector.Purpose;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.*;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.swervedrive.Vision;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.Constants;

public class AutoDrive extends Command{
    private final SwerveSubsystem swerve;
    private final limelight limelight;
    private final Vision vision;

    // Rotation2d tagRotation2d;
    // Pose2d RobotPose, tagPose;
    // Transform2d deltaTransform2d = new Transform2d();
    // Translation2d targetTranslation2d = new Translation2d();
    // Rotation2d targetRotation2d = new Rotation2d();
    // Transform2d delTransform2d = new Transform2d();
    // double deltaDeg = targetRotation2d.getDegrees();

    // private PIDController driveCtrl = new PIDController(Constants.AutoConstants.AutoDrivePIDF.P, 
    //                                                     Constants.AutoConstants.AutoDrivePIDF.I, 
    //                                                     Constants.AutoConstants.AutoDrivePIDF.D);
    // private PIDController turnCtrl = new PIDController(Constants.AutoConstants.AutoTurnPIDF.P, 
    //                                                     Constants.AutoConstants.AutoTurnPIDF.I, 
    //                                                     Constants.AutoConstants.AutoTurnPIDF.D);

    public AutoDrive(SwerveSubsystem swerve, limelight limelight, Vision vision) {
        this.swerve = swerve;
        this.limelight = limelight;
        this.vision = vision;
        addRequirements(this.swerve, this.limelight, this.vision);
    }
    @Override
    public void execute(){
        // PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("");
        // Pose2d botPose = mt2.pose;
        // Pose2d TagPose = vision.getTagPose();
        // Transform2d MeterFrom_Tag = TagPose.minus(new Pose2d(0, -1, Rotation2d.fromDegrees(0)));
        // Translation2d trans_BotToTag = MeterFrom_Tag.getTranslation();
        // Rotation2d rota_BotToTag = MeterFrom_Tag.getRotation();

        // if(Math.abs(trans_BotToTag.getX()) != 0 ){
        //     Commands.runOnce( () -> swerve.driveToPose(new Pose2d(trans_BotToTag, rota_BotToTag)), swerve);
            // Commands.runOnce( () -> swerve.drive(trans_BotToTag, rota_BotToTag.getDegrees(), false));

        Commands.runOnce( () -> swerve.resetOdometry(new Pose2d(2.601, 3.301, Rotation2d.fromDegrees(0))), swerve);
        Commands.runOnce( () -> swerve.driveToPose(new Pose2d(2.901, 4.031, Rotation2d.fromDegrees(80.786))), swerve);

        // Pose2d LLPose = LimelightHelpers.getBotPose2d("");
        
        // if (LLPose.getX() != 0) {
        //     swerve.resetOdometry(LLPose);
        //     stop = false;

        //     tagRotation2d = new Rotation2d(Math.toRadians(90));
        //     RobotPose = LimelightHelpers.getBotPose3d_TargetSpace("").toPose2d();
        //     tagPose = vision.getTagPose();
        //     deltaTransform2d = tagPose.minus(RobotPose);
        //     targetTranslation2d = deltaTransform2d.getTranslation();
        //     targetRotation2d = deltaTransform2d.getRotation();
        //     deltaDeg = targetRotation2d.getDegrees();
        // }
        // else {
        //     stop = true; 
        // }
        
        // if ((Math.abs(deltaTransform2d.getX()) < 3) && (Math.abs(deltaTransform2d.getY()) < 3) && (Math.abs(deltaDeg) < 3)){
        //     Commands.runOnce( () -> swerve.drive(delTransform2d.getTranslation(), turnCtrl.calculate(targetRotation2d.getDegrees()), false), swerve);
            swerve.drive(new Translation2d(16.315, 5.829), 142.877, false);
        //     System.out.println(deltaTransform2d.getX());
        // }
        // else {
        //     swerve.drive(targetTranslation2d, 0, true);
            // // swerve.drive(targetTranslation2d, turnCtrl.calculate(deltaDeg), false);
        // }
    }

    // @Override
    // public boolean isFinished(){
        // return stop;
    // }

    @Override
    public void end(boolean interrupted) {
    }
    
}