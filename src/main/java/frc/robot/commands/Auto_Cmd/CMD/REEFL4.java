package frc.robot.commands.Auto_Cmd.CMD;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Elevator;

public class REEFL4 extends Command {
    private final Arm arm;
    private final Coral coral;    
    private final Elevator elevator;

    public REEFL4 (Arm arm, Coral coral, Elevator elevator){
        this.arm = arm;
        this.coral = coral;
        this.elevator = elevator;
        addRequirements(this.arm, this.coral, this.elevator);
    }

    public void excute(){
        elevator.ELE_RL4();
        arm.Arm_RL4();
        
        if(coral.CoarlDetected()){
            arm.Arm_Station();
            elevator.ELE_Floor();
        }
        }
}
