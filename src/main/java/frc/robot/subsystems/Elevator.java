package frc.robot.subsystems;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

// Motor *2, 
public class Elevator extends SubsystemBase{

    private final TalonFX Left_Motor = new TalonFX(ElevatorConstants.LeftMotor_ID, "rio");
    private final TalonFX Right_Motor = new TalonFX(ElevatorConstants.RightMotor_ID, "rio");

    private final CANcoder Encoder = new CANcoder(ElevatorConstants.Encoder_ID, "rio");

    // PIDController pid = new PIDController(ElevatorConstants.P, 
    //                                       ElevatorConstants.I, 
    //                                       ElevatorConstants.D,);

    private double LastPos = 0; 
    private int rotation = 0;

    public Elevator(){
        var LeftMotorConfig = Left_Motor.getConfigurator();
        var RightMotorConfig = Right_Motor.getConfigurator();
        var EncoderConfig = Encoder.getConfigurator();

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
        // Left_Motor.setControl(new MotionMagicDutyCycle(0));
        // Right_Motor.setControl(new MotionMagicDutyCycle(0));
        // Encoder.setPosition(0);
        // rotation = 0;
    }

    public double getAbsolutePosition(){
        // return rotation * 360 + Encoder.getAbsolutePosition().getValueAsDouble();
        return Right_Motor.getPosition().getValueAsDouble();
        // return Encoder.getAbsolutePosition().getValueAsDouble();
    }
    public double getLEFTAbsolutePosition(){
        // return rotation * 360 + Encoder.getAbsolutePosition().getValueAsDouble();
        return Left_Motor.getPosition().getValueAsDouble();
        // return Encoder.getAbsolutePosition().getValueAsDouble();
    }

    public void ELE_Floor(){
        Left_Motor.setControl(new MotionMagicDutyCycle(ElevatorConstants.floor));
        Right_Motor.setControl(new MotionMagicDutyCycle(ElevatorConstants.floor));
    }

    public void ELE_RL1(){
        Left_Motor.setControl(new MotionMagicDutyCycle(ElevatorConstants.L1));
        Right_Motor.setControl(new MotionMagicDutyCycle(ElevatorConstants.L1));
    }

    public void ELE_RL2(){
        Left_Motor.setControl(new MotionMagicDutyCycle(ElevatorConstants.L2));
        Right_Motor.setControl(new MotionMagicDutyCycle(ElevatorConstants.L2));
    }

    public void ELE_RL3(){
        Left_Motor.setControl(new MotionMagicDutyCycle(ElevatorConstants.L3));
        Right_Motor.setControl(new MotionMagicDutyCycle(ElevatorConstants.L3));
    }

    public void ELE_RL4(){
        Left_Motor.setControl(new MotionMagicDutyCycle(ElevatorConstants.L4));
        Right_Motor.setControl(new MotionMagicDutyCycle(ElevatorConstants.L4));
    }

    public void ELE_Up(){
        Left_Motor.set(0.4);
        Right_Motor.set(0.4);
    }
    public void ELE_Down(){
        Left_Motor.set(-0.4);
        Right_Motor.set(-0.4);
    }

    public void ELE_Stop(){
        Left_Motor.set(0);
        Right_Motor.set(0);
    }

    @Override
    public void periodic(){

        // double currentPos = (Encoder.getAbsolutePosition().getValueAsDouble());
        // if (LastPos - currentPos >= 0.1) {
        //     rotation++;
        // } else if (LastPos - currentPos <= -0.1) {
        //     rotation--;
        // }
        // LastPos = (Encoder.getAbsolutePosition().getValueAsDouble()) ;
        // SmartDashboard.putNumber("Eleva_pos", (currentPos + rotation));
        getAbsolutePosition();
        getLEFTAbsolutePosition();
        SmartDashboard.putNumber("Eleva_pos", getAbsolutePosition());
        SmartDashboard.putNumber("Eleva_pos_L", getLEFTAbsolutePosition());

    }
}

