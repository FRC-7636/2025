package frc.robot.commands.Group_Cmd;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Elevator;

public class RL1 extends Command {
    private final Arm arm;
    private final Coral coral;    
    private final Elevator elevator;

    public RL1 (Arm arm, Coral coral, Elevator elevator){
        this.arm = arm;
        this.coral = coral;
        this.elevator = elevator;
        addRequirements(this.arm, this.coral, this.elevator);
    }

    public void excute(){
        elevator.ELE_RL2();
        arm.Arm_RL2();

        // if(coral.CoarlDetected()){
        //     arm.Arm_Station();
        //     elevator.ELE_Floor();
        // }
    }
}
