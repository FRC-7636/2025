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
    private final TalonFX Arm_Left_Motor = new TalonFX(CoralConstants.Arm_Left_Motor, "cantivore");
    private final TalonFX Arm_Right_Motor = new TalonFX(CoralConstants.Arm_Right_Motor, "cantivore");

    private final CANcoder Coral_Encoder = new CANcoder(CoralConstants.Coral_Encoder_ID, "cantivore");
    private final CANcoder Arm_Encoder = new CANcoder(CoralConstants.Arm_Encder, "cantivore");

    public Coral(){
        var CoralConfig = Coral_Motor.getConfigurator();
        var Arm_Left_Config = Arm_Left_Motor.getConfigurator();
        var Arm_Right_Config = Arm_Right_Motor.getConfigurator();

        Coral_Motor.setNeutralMode(NeutralModeValue.Brake);
        Arm_Left_Motor.setNeutralMode(NeutralModeValue.Brake);
        Arm_Right_Motor.setNeutralMode(NeutralModeValue.Brake);

        Coral_Motor.setInverted(CoralConstants.Coral_Inverted);
        Arm_Left_Motor.setInverted(CoralConstants.Arm_Left_Inverted);
        Arm_Right_Motor.setInverted(CoralConstants.Arm_Right_Inverted);

        // set feedback sensor as integrated sensor
        CoralConfig.apply(new FeedbackConfigs()
                .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor));
        Arm_Left_Config.apply(new FeedbackConfigs()
                .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor));
        Arm_Right_Config.apply(new FeedbackConfigs()
                .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor));
        
        // set maximum acceleration and velocity        
        CoralConfig.apply(new MotionMagicConfigs()
                .withMotionMagicAcceleration(ElevatorConstants.MAX_ACCEL)
                .withMotionMagicCruiseVelocity(ElevatorConstants.MAX_VELOCITY));
        Arm_Left_Config.apply(new MotionMagicConfigs()
                .withMotionMagicAcceleration(ElevatorConstants.MAX_ACCEL)
                .withMotionMagicCruiseVelocity(ElevatorConstants.MAX_VELOCITY));
        Arm_Right_Config.apply(new MotionMagicConfigs()
                .withMotionMagicAcceleration(ElevatorConstants.MAX_ACCEL)
                .withMotionMagicCruiseVelocity(ElevatorConstants.MAX_VELOCITY));


        // Sets the mechanism position of the device in mechanism rotations.
        CoralConfig.setPosition(0);
        Arm_Left_Config.setPosition(0);
        Arm_Right_Config.setPosition(0);

        // PIDConfig
        Slot0Configs PIDConfig = new Slot0Configs();
        PIDConfig.kP = ElevatorConstants.P;
        PIDConfig.kI = ElevatorConstants.I;
        PIDConfig.kD = ElevatorConstants.D;
        PIDConfig.kV = ElevatorConstants.F;
        CoralConfig.apply(PIDConfig);
        Arm_Left_Config.apply(PIDConfig);
        Arm_Right_Config.apply(PIDConfig);
    }

    public void position(){
        Coral_Motor.setControl(new MotionMagicDutyCycle(0));
    }

    public double getCoralAbsPos(){
        return Coral_Encoder.getAbsolutePosition().getValueAsDouble();
    }

    public double getArmAbsPos(){
        return Arm_Encoder.getAbsolutePosition().getValueAsDouble();
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

    // Coral Arm
    public void Arm_Station(){
        Arm_Left_Motor.setControl(new MotionMagicDutyCycle(CoralConstants.Arm_Station));
        Arm_Right_Motor.setControl(new MotionMagicDutyCycle(CoralConstants.Arm_Station));
    }

    public void Arm_Reef(){
        Arm_Left_Motor.setControl(new MotionMagicDutyCycle(CoralConstants.Arm_Reef));
        Arm_Right_Motor.setControl(new MotionMagicDutyCycle(CoralConstants.Arm_Reef));
    }

    public void Arm_SwingUP(){
        Arm_Left_Motor.set(0.3);
        Arm_Right_Motor.set(0.3);
    }

    public void Arm_SwingDOWN(){
        Arm_Left_Motor.set(0.3);
        Arm_Right_Motor.set(0.3);
    }

    @Override
    public void periodic(){
        getCoralAbsPos();
        getArmAbsPos();
    }
}
