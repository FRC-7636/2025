package frc.robot.commands.Auto_Cmd.CMD;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Elevator;

public class CoralSation extends Command{
    private final Arm arm;
    private final Coral coral;
    private final Elevator elevator;

    public CoralSation(Arm arm, Coral coral, Elevator elevator){
        this.arm = arm;
        this.coral = coral;
        this.elevator = elevator;
        addRequirements(this.arm, this.coral, this.elevator);
    }

    public void excute(){
        arm.Arm_Station();
        new WaitCommand(0.1);
        coral.Coral_Suck();
    }
    
}
