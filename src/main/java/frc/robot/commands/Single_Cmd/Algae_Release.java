package frc.robot.commands.Single_Cmd;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Algae;

public class Algae_Release extends Command{
    private final Algae algae;

    public Algae_Release(Algae algae){
        this.algae = algae;
        addRequirements(this.algae);
    }

    public void excute(){
        algae.Intake_out();
        algae.shoot();
        new WaitCommand(0.5);

        algae.Stop();
        algae.Intake_back();
    }
    
}
