package frc.robot.commands.Auto;

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

public class AutoToReef extends Command{
    private final SwerveSubsystem swerve;
    private final limelight limelight;
    private final Vision vision;

    public AutoToReef(SwerveSubsystem swerve, limelight limelight, Vision vision) {
        this.swerve = swerve;
        this.limelight = limelight;
        this.vision = vision;
        addRequirements(this.swerve, this.limelight, this.vision);
    }
    @Override
    public void execute(){
        Commands.runOnce( () -> swerve.resetOdometry(new Pose2d(2.601, 3.301, Rotation2d.fromDegrees(0))), swerve);
        Commands.runOnce( () -> swerve.driveToPose(new Pose2d(2.901, 4.031, Rotation2d.fromDegrees(80.786))), swerve);
    }

    @Override
    public void end(boolean interrupted) {
    }
    
}