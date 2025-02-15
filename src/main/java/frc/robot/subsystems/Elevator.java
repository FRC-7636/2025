package frc.robot.subsystems;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

// Motor *2, 
public class Elevator extends SubsystemBase{

    private final TalonFX Left_Motor = new TalonFX(ElevatorConstants.LeftMotor_ID, "cantivore");
    private final TalonFX Right_Motor = new TalonFX(ElevatorConstants.RightMotor_ID, "cantivore");

    private final CANcoder Encoder = new CANcoder(ElevatorConstants.Encoder_ID, "cantivore");


    public Elevator(){
        var LeftMotorConfig = Left_Motor.getConfigurator();
        var RightMotorConfig = Right_Motor.getConfigurator();

        Left_Motor.setNeutralMode(NeutralModeValue.Brake);
        Right_Motor.setNeutralMode(NeutralModeValue.Brake);

        Left_Motor.setInverted(ElevatorConstants.LeftMotor_Inverted);
        Right_Motor.setInverted(ElevatorConstants.RightMotor_Inverted);

        // set feedback sensor as integrated sensor
        LeftMotorConfig.apply(new FeedbackConfigs()
                .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor));
        RightMotorConfig.apply(new FeedbackConfigs()
                .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor));

        // set maximum acceleration and velocity
        LeftMotorConfig.apply(new MotionMagicConfigs()
                .withMotionMagicAcceleration(ElevatorConstants.MAX_ACCEL)
                .withMotionMagicCruiseVelocity(ElevatorConstants.MAX_VELOCITY));

        RightMotorConfig.apply(new MotionMagicConfigs()
                .withMotionMagicAcceleration(ElevatorConstants.MAX_ACCEL)
                .withMotionMagicCruiseVelocity(ElevatorConstants.MAX_VELOCITY));

        // Sets the mechanism position of the device in mechanism rotations.
        LeftMotorConfig.setPosition(0);
        RightMotorConfig.setPosition(0);

        // PIDConfig
        Slot0Configs PIDConfig = new Slot0Configs();
        PIDConfig.kP = ElevatorConstants.P;
        PIDConfig.kI = ElevatorConstants.I;
        PIDConfig.kD = ElevatorConstants.D;
        PIDConfig.kV = ElevatorConstants.F;
        LeftMotorConfig.apply(PIDConfig);
        RightMotorConfig.apply(PIDConfig);
    }

    public void position(){
        Left_Motor.setControl(new MotionMagicDutyCycle(0));
        Right_Motor.setControl(new MotionMagicDutyCycle(0));
    }

    public double getAbsolutePosition(){
        return Encoder.getAbsolutePosition().getValueAsDouble();
    }

    public void setFloor(){
        Left_Motor.setControl(new MotionMagicDutyCycle(ElevatorConstants.floor));
        Right_Motor.setControl(new MotionMagicDutyCycle(ElevatorConstants.floor));
    }

    public void setRL1(){
        Left_Motor.setControl(new MotionMagicDutyCycle(ElevatorConstants.L1));
        Right_Motor.setControl(new MotionMagicDutyCycle(ElevatorConstants.L1));
    }

    public void setRL2(){
        Left_Motor.setControl(new MotionMagicDutyCycle(ElevatorConstants.L2));
        Right_Motor.setControl(new MotionMagicDutyCycle(ElevatorConstants.L2));
    }

    public void setRL3(){
        Left_Motor.setControl(new MotionMagicDutyCycle(ElevatorConstants.L3));
        Right_Motor.setControl(new MotionMagicDutyCycle(ElevatorConstants.L3));
    }

    public void setRL4(){
        Left_Motor.setControl(new MotionMagicDutyCycle(ElevatorConstants.L4));
        Right_Motor.setControl(new MotionMagicDutyCycle(ElevatorConstants.L4));
    }

    public void Eleva_Up(){
        Left_Motor.set(0.3);
        Right_Motor.set(0.3);
    }

    public void Eleva_Down(){
        Left_Motor.set(-0.3);
        Right_Motor.set(-0.3);
    }

     @Override
     public void periodic(){    
        SmartDashboard.putNumber("Eleva_pos", getAbsolutePosition());
     }
}

