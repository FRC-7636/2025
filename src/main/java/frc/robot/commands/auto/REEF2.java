package frc.robot.commands.Auto;

import java.lang.ModuleLayer.Controller;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.limelight;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class REEF2 extends Command{
    private final SwerveSubsystem swerve;
    private final limelight limelight;
    PIDController controller = new PIDController(5, 0, 0);
    public boolean stop;
   
    public REEF2(SwerveSubsystem swerve, limelight limelight){
        this.swerve = swerve;
        this.limelight = limelight;
        addRequirements(this.swerve, this.limelight);
    }

    public void excute(){
        if(Math.abs(LimelightHelpers.getTargetPose3d_RobotSpace("").getRotation().getY()) < 1 ){
            controller.enableContinuousInput(-Math.PI, Math.PI);
            swerve.aim();
            stop = false;
        }
        else{
            stop = true;
        }
    }   

    @Override
    public boolean isFinished(){
        return stop;
    }

    @Override
    public void end(boolean interrupted) {
    }
}