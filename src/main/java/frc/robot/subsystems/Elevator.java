package frc.robot.subsystems;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase{

    private final TalonFX LeftMotor = new TalonFX(ElevatorConstants.LeftMotor_ID, "");
    private final TalonFX RightMotor = new TalonFX(ElevatorConstants.RightMotor_ID, "");

    public Elevator(){
        var LeftMotorConfig = LeftMotor.getConfigurator();
        var RightMotorConfig = RightMotor.getConfigurator();

        LeftMotor.setNeutralMode(NeutralModeValue.Brake);
        LeftMotor.setInverted(false);
        RightMotor.setNeutralMode(NeutralModeValue.Brake);
        RightMotor.setInverted(false);

        // set feedback sensor as integrated sensor
        LeftMotorConfig.apply(new FeedbackConfigs()
                .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor));
        RightMotorConfig.apply(new FeedbackConfigs()
                .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor));
    }
    
}
