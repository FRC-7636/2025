package frc.robot.subsystems;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralConstants;
import frc.robot.Constants.ElevatorConstants;

// Motor * 1
public class Coral extends SubsystemBase{
    private final TalonFX CoralMotor = new TalonFX(CoralConstants.CoralMotor_ID, "cantivore");
    private final CANcoder CoralEncoder = new CANcoder(CoralConstants.CoralEncoder_ID, "cantivore");

    public Coral(){
        var coralConfig = CoralMotor.getConfigurator();

        CoralMotor.setNeutralMode(NeutralModeValue.Brake);
        CoralMotor.setInverted(CoralConstants.coral_Inverted);

        // set feedback sensor as integrated sensor
        coralConfig.apply(new FeedbackConfigs()
                .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor));
        
        // set maximum acceleration and velocity        
        coralConfig.apply(new MotionMagicConfigs()
                .withMotionMagicAcceleration(ElevatorConstants.MAX_ACCEL)
                .withMotionMagicCruiseVelocity(ElevatorConstants.MAX_VELOCITY));

        // Sets the mechanism position of the device in mechanism rotations.
        coralConfig.setPosition(0);

        // PIDConfig
        Slot0Configs PIDConfig = new Slot0Configs();
        PIDConfig.kP = ElevatorConstants.P;
        PIDConfig.kI = ElevatorConstants.I;
        PIDConfig.kD = ElevatorConstants.D;
        PIDConfig.kV = ElevatorConstants.F;
        coralConfig.apply(PIDConfig);
        coralConfig.apply(PIDConfig);
    }

    public void position(){
        CoralMotor.setControl(new MotionMagicDutyCycle(0));
    }

    public double getAbsolutePosition(){
        return CoralEncoder.getAbsolutePosition().getValueAsDouble();
    }

    public void open(){
        CoralMotor.setControl(new MotionMagicDutyCycle(CoralConstants.Coral_Open));
    }

    public void close(){
        CoralMotor.setControl(new MotionMagicDutyCycle(CoralConstants.Coral_Close));
    }
}
