package frc.robot.commands.Auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class AutoToReef extends Command{
    private final SwerveSubsystem swerve;

    public AutoToReef(SwerveSubsystem swerve) {
        this.swerve = swerve;
        addRequirements(this.swerve);
    }

    @Override
    public void execute(){
        Commands.runOnce( () -> swerve.resetOdometry(new Pose2d(2.601, 3.301, Rotation2d.fromDegrees(0))), swerve);
        Commands.runOnce( () -> swerve.driveToPose(new Pose2d(2.901, 4.031, Rotation2d.fromDegrees(0))), swerve);
        // Commands.runOnce( () -> swerve.driveToPose(new Pose2d(2.901, 4.031, Rotation2d.fromDegrees(80.786))), swerve);
        // Commands.runOnce( () -> swerve.driveToPose(new Pose2d(2.901, 4.031, Rotation2d.fromDegrees(260.786))), swerve);

        // Commands.runOnce( () -> swerve.resetOdometry(new Pose2d(2.7151, 5.351, Rotation2d.fromDegrees(0))), swerve);
        // Commands.runOnce( () -> swerve.driveToPose(new Pose2d(3.7151, 5.351, Rotation2d.fromDegrees(80.786))), swerve);
    }

    @Override
    public void end(boolean interrupted) {
    }
    
}