package frc.robot.commands.Auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.limelight;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.swervedrive.Vision;
import swervelib.SwerveInputStream;

public class Reef extends SequentialCommandGroup{

    PIDController controller = new PIDController(1, 0, 0);
    

    public Reef(SwerveSubsystem swerve, limelight limelight, Vision vision){
        addRequirements(swerve);
        addCommands(Commands.runOnce(() -> controller.enableContinuousInput(-Math.PI, Math.PI), swerve));
        addCommands(Commands.runOnce(() -> {swerve.aim();}, swerve));
        
        addCommands(Commands.runOnce(() -> vision.getPoseFromTag(), vision));
    }
    
}
