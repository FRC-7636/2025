package frc.robot.commands.Group_Cmd;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Coral;

public class Coral_Station extends Command{
    private final Arm arm;
    private final Coral coral;

    public Coral_Station(Arm arm, Coral coral){
        this.arm = arm;
        this.coral = coral;
        addRequirements(this.arm, this.coral);
    }

    public void excute(){
        arm.Arm_Coral_Sation();
        coral.Coral_Suck();
        new WaitCommand(0.5);
        
        coral.Coral_Stop();
        arm.Arm_Coral_StartUp();
    }
    
}
