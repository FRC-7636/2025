package frc.robot.commands.Single_Cmd;

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
        algae.Intake_out();
        algae.suck();
        new WaitCommand(0.3);

        algae.Stop();
    }
}
