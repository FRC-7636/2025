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
    private final TalonFX Left_Motor = new TalonFX(ClimberConstants.Left_Motor_ID, "cantivore");
    private final TalonFX Right_Motor = new TalonFX(ClimberConstants.RightMotor_ID, "cantivore");

    private final CANcoder Encoder = new CANcoder(ClimberConstants.Encoder_ID, "cantivore");

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
        var Left_Motor_Config = Left_Motor.getConfigurator();
        var Right_Motor_Config = Right_Motor.getConfigurator();

        Left_Motor.setNeutralMode(NeutralModeValue.Brake);
        Right_Motor.setNeutralMode(NeutralModeValue.Brake);

        Left_Motor.setInverted(ClimberConstants.LeftMotor_Inverted);
        Right_Motor.setInverted(ClimberConstants.RightMotor_Inverted);

        // set feedback sensor as integrated sensor
        Left_Motor_Config.apply(new FeedbackConfigs()
                .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor));
        Right_Motor_Config.apply(new FeedbackConfigs()
                .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor));

        // set maximum acceleration and velocity
        Left_Motor_Config.apply(new MotionMagicConfigs()
                .withMotionMagicAcceleration(ElevatorConstants.MAX_ACCEL)
                .withMotionMagicCruiseVelocity(ElevatorConstants.MAX_VELOCITY));

        Right_Motor_Config.apply(new MotionMagicConfigs()
                .withMotionMagicAcceleration(ElevatorConstants.MAX_ACCEL)
                .withMotionMagicCruiseVelocity(ElevatorConstants.MAX_VELOCITY));

        // Sets the mechanism position of the device in mechanism rotations.
        Left_Motor_Config.setPosition(0);
        Right_Motor_Config.setPosition(0);

        // PIDConfig
        Slot0Configs PIDConfig = new Slot0Configs();
        PIDConfig.kP = ClimberConstants.P;
        PIDConfig.kI = ClimberConstants.I;
        PIDConfig.kD = ClimberConstants.D;
        PIDConfig.kV = ClimberConstants.F;
        Left_Motor_Config.apply(PIDConfig);
        Right_Motor_Config.apply(PIDConfig);
    }

    public double getAbsolutePosition(){
        return Encoder.getAbsolutePosition().getValueAsDouble();
    }

    public void Climb(){
        Left_Motor.setControl(new MotionMagicDutyCycle(ClimberConstants.Climb_Angle));
        Right_Motor.setControl(new MotionMagicDutyCycle(ClimberConstants.Climb_Angle));
    }

    public void Up(){
        Left_Motor.set(0.3);
        Right_Motor.set(0.3);
    }

    public void Down(){
        Left_Motor.set(0.3);
        Right_Motor.set(0.3);
    }
    @Override 
    public void periodic(){
        SmartDashboard.putNumber("Climb_Pos", getAbsolutePosition());
    }
}
