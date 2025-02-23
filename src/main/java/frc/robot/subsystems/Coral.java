package frc.robot.subsystems;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.Constants.CoralConstants;
import frc.robot.Constants.ElevatorConstants;

// Kraken Motor - for 
// 2 Falcon 500 Motor - for
public class Coral extends SubsystemBase{
    private final DigitalInput Coral_Detect = new DigitalInput(CoralConstants.Coral_Sensor_ID);

    private final TalonFX Coral_Motor = new TalonFX(CoralConstants.Coral_Motor_ID, "XiuBengBai");

    public Coral(){
        // var CoralConfig = Coral_Motor.getConfigurator();

        Coral_Motor.setNeutralMode(NeutralModeValue.Brake);

        Coral_Motor.setInverted(CoralConstants.Coral_Inverted);

        // // set feedback sensor as integrated sensor
        // CoralConfig.apply(new FeedbackConfigs()
        //         .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor));
        
        // // set maximum acceleration and velocity        
        // CoralConfig.apply(new MotionMagicConfigs()
        //         .withMotionMagicAcceleration(ElevatorConstants.MAX_ACCEL)
        //         .withMotionMagicCruiseVelocity(ElevatorConstants.MAX_VELOCITY));
    
        // // Sets the mechanism position of the device in mechanism rotations.
        // CoralConfig.setPosition(0);
        
        // // PIDConfig
        // Slot0Configs PIDConfig = new Slot0Configs();
        // PIDConfig.kP = ElevatorConstants.P;
        // PIDConfig.kI = ElevatorConstants.I;
        // PIDConfig.kD = ElevatorConstants.D;
        // PIDConfig.kV = ElevatorConstants.F;
        // CoralConfig.apply(PIDConfig);
    }

    /**
     * Get the sensor value of the sensor.
     * @return Whether the digital sensor is detecting an Algae.
     */
    public boolean CoarlDetected() {
        return !Coral_Detect.get();
    }

    // public void position(){
    //     Coral_Motor.setControl(new MotionMagicDutyCycle(0));
    // }

    // public double getCoralAbsPos(){
    //     return Coral_Encoder.getAbsolutePosition().getValueAsDouble();
    // }

    // Coral Intake
    public void Coral_Suck(){
        Coral_Motor.set(0.3);
    }

    public void Coral_Shoot(){
        Coral_Motor.set(-0.3);
    }

    public void Coral_Stop(){
        Coral_Motor.set(0);
    }

    @Override
    public void periodic(){
        SmartDashboard.putBoolean("Algae_Detected", CoarlDetected());
    }
}
