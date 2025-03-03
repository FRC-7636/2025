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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase{
    private final TalonFX Arm_Motor = new TalonFX(ArmConstants.Arm_ID, "mech");

    private final CANcoder Arm_Encoder = new CANcoder(ArmConstants.Arm_Encoder_ID, "mech");

    private double Arm_Pos = 0;
    private double LastPos = 0;
    private int rotation = 0;

    public Arm(){
        var Arm_Motor_Config = Arm_Motor.getConfigurator();

        Arm_Motor.setNeutralMode(NeutralModeValue.Brake);

        Arm_Motor.setInverted(ArmConstants.Arm_Inverted);

        // set feedback sensor as integrated sensor
        Arm_Motor_Config.apply(new FeedbackConfigs()
                .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder).withFeedbackRemoteSensorID(ArmConstants.Arm_Encoder_ID));

        // set maximum acceleration and velocity        
        Arm_Motor_Config.apply(new MotionMagicConfigs()
                .withMotionMagicAcceleration(ArmConstants.MAX_ACCEL)
                .withMotionMagicCruiseVelocity(ArmConstants.MAX_VELOCITY));

        // Sets the mechanism position of the device in mechanism rotations.
        Arm_Motor_Config.setPosition(0);
        
        // Arm PIDConfig
        Slot0Configs Arm_PIDConfig = new Slot0Configs();
        Arm_PIDConfig.kP = ArmConstants.Arm_P;
        Arm_PIDConfig.kI = ArmConstants.Arm_I;
        Arm_PIDConfig.kD = ArmConstants.Arm_D;
        Arm_PIDConfig.kV = ArmConstants.Arm_F;
        Arm_Motor_Config.apply(Arm_PIDConfig);
    }

    public double getArmPos(){
        return Arm_Encoder.getAbsolutePosition().getValueAsDouble();
        // return Arm_Motor.getPosition().getValueAsDouble();
    }

    // public double getstat(){
    //     return Arm_Motor.
    // }

    // Arm 
    public void Arm_Zero(){
        Arm_Motor.setControl(new MotionMagicDutyCycle(ArmConstants.Arm_Zero));
    }

    public void Arm_StartUp(){
        Arm_Motor.setControl(new MotionMagicDutyCycle(ArmConstants.Arm_StartUp));
    }

    public void Arm_Station(){
        Arm_Motor.setControl(new MotionMagicDutyCycle(ArmConstants.Arm_Station));
    }

    public void Arm_RL1(){
        Arm_Motor.setControl(new MotionMagicDutyCycle(ArmConstants.Arm_RL1));
    }

    public void Arm_RL2(){
        Arm_Motor.setControl(new MotionMagicDutyCycle(25));
    }

    public void Arm_RL3(){
        Arm_Motor.setControl(new MotionMagicDutyCycle(ArmConstants.Arm_RL3));
    }

    public void Arm_RL4(){
        Arm_Motor.setControl(new MotionMagicDutyCycle(ArmConstants.Arm_RL4));
    }

    public void Arm_DOWN(){
        Arm_Motor.set(-0.3);
    }

    public void Arm_UP(){
        Arm_Motor.set(0.3);
    }

    public void Arm_Stop(){
        Arm_Motor.set(0);
    }

    @Override
    public void periodic(){
        getArmPos();
        SmartDashboard.putNumber("Arm_Pos", getArmPos());

        double currentPos = (Arm_Encoder.getAbsolutePosition().getValueAsDouble());
        if (LastPos - currentPos >= 0.1) {
            rotation++;
        } else if (LastPos - currentPos <= -0.1) {
            rotation--;
        }
        LastPos = (Arm_Encoder.getAbsolutePosition().getValueAsDouble()) ;
        SmartDashboard.putNumber("Arm_Calculated", (currentPos + rotation));
    }
}
