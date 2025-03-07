package frc.robot.commands.Group_Cmd;

import java.security.interfaces.EdECKey;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Elevator;

public class Coral_Station extends Command{
    private final Arm arm;
    private final Coral coral;
    private final Elevator elevator;

    public Coral_Station(Arm arm, Coral coral, Elevator elevator){
        this.arm = arm;
        this.coral = coral;
        this.elevator = elevator;
        addRequirements(this.arm, this.coral, this.elevator);
    }

    public void excute(){
        arm.Arm_Station();
    }
    
}
