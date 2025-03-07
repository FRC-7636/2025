package frc.robot.commands.Single_Cmd;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class Climb extends Command{
    private final Climber climber;

    public Climb(Climber climber){
        this.climber = climber;
        addRequirements(this.climber);
    }

    public void excute(){
        climber.Climb_Zero();
    }
}
