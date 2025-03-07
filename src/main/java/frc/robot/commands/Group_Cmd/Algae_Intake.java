package frc.robot.commands.Group_Cmd;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Algae;

public class Algae_Intake extends Command{
    private final Algae algae;

    public Algae_Intake(Algae algae){
        this.algae = algae;
        addRequirements(this.algae);
    }
    
    public void excute(){
        algae.Algae_out();
    }
}
