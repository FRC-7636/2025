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

// Kraken Motor - for 
// 2 Falcon 500 Motor - for
public class Coral extends SubsystemBase{
    private final TalonFX Coral_Motor = new TalonFX(CoralConstants.Coral_Motor_ID, "cantivore");

    private final CANcoder Coral_Encoder = new CANcoder(CoralConstants.Coral_Encoder_ID, "cantivore");

    public Coral(){
        var CoralConfig = Coral_Motor.getConfigurator();

        Coral_Motor.setNeutralMode(NeutralModeValue.Brake);

        Coral_Motor.setInverted(CoralConstants.Coral_Inverted);

        // set feedback sensor as integrated sensor
        CoralConfig.apply(new FeedbackConfigs()
                .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor));
        
        // set maximum acceleration and velocity        
        CoralConfig.apply(new MotionMagicConfigs()
                .withMotionMagicAcceleration(ElevatorConstants.MAX_ACCEL)
                .withMotionMagicCruiseVelocity(ElevatorConstants.MAX_VELOCITY));
    
        // Sets the mechanism position of the device in mechanism rotations.
        CoralConfig.setPosition(0);
        
        // PIDConfig
        Slot0Configs PIDConfig = new Slot0Configs();
        PIDConfig.kP = ElevatorConstants.P;
        PIDConfig.kI = ElevatorConstants.I;
        PIDConfig.kD = ElevatorConstants.D;
        PIDConfig.kV = ElevatorConstants.F;
        CoralConfig.apply(PIDConfig);
    }

    public void position(){
        Coral_Motor.setControl(new MotionMagicDutyCycle(0));
    }

    public double getCoralAbsPos(){
        return Coral_Encoder.getAbsolutePosition().getValueAsDouble();
    }

    // Coral Intake
    public void Coral_Open(){
        Coral_Motor.setControl(new MotionMagicDutyCycle(CoralConstants.Coral_Open));
    }

    public void Coral_Close(){
        Coral_Motor.setControl(new MotionMagicDutyCycle(CoralConstants.Coral_Close));
    }

    public void Coral_Suck(){
        Coral_Motor.set(0.3);
    }

    public void Coral_Shoot(){
        Coral_Motor.set(-0.3);
    }

    @Override
    public void periodic(){
        getCoralAbsPos();
    }
}
