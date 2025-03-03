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
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.ElevatorConstants;

// Motor * 2
public class Climber extends SubsystemBase{
    private final TalonFX Climber_Motor = new TalonFX(ClimberConstants.LeftMotor_ID, "mech");

    private final CANcoder Encoder = new CANcoder(ClimberConstants.Encoder_ID, "mech");

    // private final AbsoluteEncoder Encoder = new AbsoluteEncoder(){
    //     @Override
    //     public double getPosition() {
    //         return 0;
    //     }
    //     public double getVelocity() {
    //         return 0;
    //     }
    // };

    public Climber(){
        var Left_Motor_Config = Climber_Motor.getConfigurator();

        Climber_Motor.setNeutralMode(NeutralModeValue.Brake);

        Climber_Motor.setInverted(ClimberConstants.LeftMotor_Inverted);

        // set feedback sensor as integrated sensor
        Left_Motor_Config.apply(new FeedbackConfigs()
                .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor));

        // set maximum acceleration and velocity
        Left_Motor_Config.apply(new MotionMagicConfigs()
                .withMotionMagicAcceleration(ElevatorConstants.MAX_ACCEL)
                .withMotionMagicCruiseVelocity(ElevatorConstants.MAX_VELOCITY));
        Left_Motor_Config.setPosition(0);

        // PIDConfig
        Slot0Configs PIDConfig = new Slot0Configs();
        PIDConfig.kP = ClimberConstants.P;
        PIDConfig.kI = ClimberConstants.I;
        PIDConfig.kD = ClimberConstants.D;
        PIDConfig.kV = ClimberConstants.F;
        Left_Motor_Config.apply(PIDConfig);
    }

    public double getAbsolutePosition(){
        return Encoder.getAbsolutePosition().getValueAsDouble();
    }

    public void Climb(){
        Climber_Motor.setControl(new MotionMagicDutyCycle(ClimberConstants.Climb_Angle));
    }

    public void Climb_Zero(){
        Climber_Motor.setControl(new MotionMagicDutyCycle(ClimberConstants.Climb_Zero));
    }

    public void Up(){
        Climber_Motor.set(0.3);
    }

    public void Down(){
        Climber_Motor.set(0.3);
    }
    @Override 
    public void periodic(){
        SmartDashboard.putNumber("Climb_Pos", getAbsolutePosition());
    }
}
