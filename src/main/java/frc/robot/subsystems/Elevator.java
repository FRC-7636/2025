package frc.robot.subsystems;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.thethriftybot.ThriftyNova.PIDConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

// Motor *2, 
public class Elevator extends SubsystemBase{

    private final TalonFX LeftMotor = new TalonFX(ElevatorConstants.LeftMotor_ID, "");
    private final TalonFX RightMotor = new TalonFX(ElevatorConstants.RightMotor_ID, "");

    private final CANcoder LeftEncoder = new CANcoder(4, "cantivore");
    private final CANcoder RightEncoder = new CANcoder(ElevatorConstants.RightEncoder_ID, "cantivore");


    public Elevator(){
        var LeftMotorConfig = LeftMotor.getConfigurator();
        var RightMotorConfig = RightMotor.getConfigurator();

        LeftMotor.setNeutralMode(NeutralModeValue.Brake);
        RightMotor.setNeutralMode(NeutralModeValue.Brake);

        LeftMotor.setInverted(ElevatorConstants.LeftMotor_Inverted);
        RightMotor.setInverted(ElevatorConstants.RightMotor_Inverted);

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
        LeftMotor.setControl(new MotionMagicDutyCycle(0));
        RightMotor.setControl(new MotionMagicDutyCycle(0));
    }

    public double getAbsolutePosition(){
        return LeftEncoder.getAbsolutePosition().getValueAsDouble();
    }

    public void setFloor(){
        LeftMotor.setControl(new MotionMagicDutyCycle(ElevatorConstants.floor));
        RightMotor.setControl(new MotionMagicDutyCycle(ElevatorConstants.floor));
    }

    public void setL1(){
        LeftMotor.setControl(new MotionMagicDutyCycle(ElevatorConstants.L1));
        RightMotor.setControl(new MotionMagicDutyCycle(ElevatorConstants.L1));
    }

    public void setL2(){
        LeftMotor.setControl(new MotionMagicDutyCycle(ElevatorConstants.L2));
        RightMotor.setControl(new MotionMagicDutyCycle(ElevatorConstants.L2));
    }

    public void setL3(){
        LeftMotor.setControl(new MotionMagicDutyCycle(ElevatorConstants.L3));
        RightMotor.setControl(new MotionMagicDutyCycle(ElevatorConstants.L3));
    }

    public void setL4(){
        LeftMotor.setControl(new MotionMagicDutyCycle(ElevatorConstants.L4));
        RightMotor.setControl(new MotionMagicDutyCycle(ElevatorConstants.L4));
    }

    public void Up(){
        LeftMotor.set(0.3);
        RightMotor.set(0.3);
    }

    public void Down(){
        LeftMotor.set(-0.3);
        RightMotor.set(-0.3);
    }

     @Override
     public void periodic(){    
        SmartDashboard.putNumber("Eleva_pos", getAbsolutePosition());
     }
}

