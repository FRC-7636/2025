package frc.robot.subsystems;

import javax.xml.crypto.dsig.keyinfo.RetrievalMethod;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.CoralConstants;
import frc.robot.Constants.ElevatorConstants;

public class Arm extends SubsystemBase{
    private final TalonFX Arm_Motor = new TalonFX(ArmConstants.Arm_ID, "cantivore");
    private final TalonFX Arm_Coral_Motor = new TalonFX(ArmConstants.Arm_Coral_ID, "cantivore");

    private final CANcoder Arm_Encoder = new CANcoder(ArmConstants.Arm_Encoder_ID, "cantivore");
    private final CANcoder Arm_Coral_Encoder = new CANcoder(ArmConstants.Arm_Coral_Encoder, "cantivor");

    public Arm(){
        var Arm_Motor_Config = Arm_Motor.getConfigurator();
        var Arm_Coral_Motor_Config = Arm_Coral_Motor.getConfigurator();

        Arm_Motor.setNeutralMode(NeutralModeValue.Brake);
        Arm_Coral_Motor.setNeutralMode(NeutralModeValue.Brake);

        Arm_Motor.setInverted(ArmConstants.Arm_Inverted);
        Arm_Coral_Motor.setInverted(ArmConstants.Arm_Coral_Inverted);

        // set feedback sensor as integrated sensor
        Arm_Motor_Config.apply(new FeedbackConfigs()
                .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor));
        Arm_Coral_Motor_Config.apply(new FeedbackConfigs()
                .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor));        

        // set maximum acceleration and velocity        
        Arm_Motor_Config.apply(new MotionMagicConfigs()
                .withMotionMagicAcceleration(ElevatorConstants.MAX_ACCEL)
                .withMotionMagicCruiseVelocity(ElevatorConstants.MAX_VELOCITY));
        Arm_Coral_Motor_Config.apply(new MotionMagicConfigs()
                .withMotionMagicAcceleration(ElevatorConstants.MAX_ACCEL)
                .withMotionMagicCruiseVelocity(ElevatorConstants.MAX_VELOCITY));
    
        // Sets the mechanism position of the device in mechanism rotations.
        Arm_Motor_Config.setPosition(0);
        Arm_Coral_Motor_Config.setPosition(0);
        
        // PIDConfig
        Slot0Configs Arm_PIDConfig = new Slot0Configs();
        Arm_PIDConfig.kP = ArmConstants.Arm_P;
        Arm_PIDConfig.kI = ArmConstants.Arm_I;
        Arm_PIDConfig.kD = ArmConstants.Arm_D;
        Arm_PIDConfig.kV = ArmConstants.Arm_F;
        Arm_Motor_Config.apply(Arm_PIDConfig);

        Slot0Configs Coral_PIDConfig = new Slot0Configs();
        Arm_PIDConfig.kP = ArmConstants.Arm_Coral_P;
        Arm_PIDConfig.kI = ArmConstants.Arm_Coral_I;
        Arm_PIDConfig.kD = ArmConstants.Arm_Coral_D;
        Arm_PIDConfig.kV = ArmConstants.Arm_Coral_F;
        Arm_Coral_Motor_Config.apply(Coral_PIDConfig);
    }

    public double getArmPos(){
        return Arm_Encoder.getAbsolutePosition().getValueAsDouble();
    }

    public double getArmCoralPos(){
        return Arm_Coral_Encoder.getAbsolutePosition().getValueAsDouble();
    }

    // Arm 
    public void Arm_StartUp(){
        Arm_Motor.setControl(new MotionMagicDutyCycle(ArmConstants.Arm_Coral_StartUp));
    }

    public void Arm_Station(){
        Arm_Motor.setControl(new MotionMagicDutyCycle(ArmConstants.Arm_Coral_Station));
    }

    public void Arm_RL1(){
        Arm_Motor.setControl(new MotionMagicDutyCycle(ArmConstants.Arm_RL1));
    }

    public void Arm_RL2(){
        Arm_Motor.setControl(new MotionMagicDutyCycle(ArmConstants.Arm_RL2));
    }

    public void Arm_RL3(){
        Arm_Motor.setControl(new MotionMagicDutyCycle(ArmConstants.Arm_RL3));
    }

    public void Arm_RL4(){
        Arm_Motor.setControl(new MotionMagicDutyCycle(ArmConstants.Arm_RL4));
    }

    public void Arm_UP(){
        Arm_Motor.set(0.3);
    }

    public void Arm_DOWN(){
        Arm_Motor.set(0.3);
    }

    // Arm Coral 
    public void Arm_Coral_StartUp(){
        Arm_Motor.setControl(new MotionMagicDutyCycle(ArmConstants.Arm_Coral_StartUp));
    }

    public void Arm_Coral_Sation(){
        Arm_Motor.setControl(new MotionMagicDutyCycle(ArmConstants.Arm_Coral_Station));
    }

    public void Arm_Coral_RL1(){
        Arm_Motor.setControl(new MotionMagicDutyCycle(ArmConstants.Arm_Coral_RL1));
    }

    public void Arm_Coral_RL2(){
        Arm_Motor.setControl(new MotionMagicDutyCycle(ArmConstants.Arm_Coral_RL2));
    }

    public void Arm_Coral_RL3(){
        Arm_Motor.setControl(new MotionMagicDutyCycle(ArmConstants.Arm_Coral_RL3));
    }

    public void Arm_Coral_RL4(){
        Arm_Motor.setControl(new MotionMagicDutyCycle(ArmConstants.Arm_Coral_RL4));
    }

    public void Arm_Coral_UP(){
        Arm_Motor.set(0.3);
    }

    public void Arm_Coral_DOWN(){
        Arm_Motor.set(0.3);
    }
    @Override
    public void periodic(){
        getArmPos();
        getArmCoralPos();
    }
}
