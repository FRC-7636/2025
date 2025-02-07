package frc.robot.subsystems;

import org.dyn4j.collision.broadphase.StaticValueAABBExpansionMethod;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

// NEO Motor - for 
// Karken Motor - for 
public class Algae extends SubsystemBase {
    private final SparkMax ShuShu = new SparkMax(31, MotorType.kBrushless);
    private final SparkClosedLoopController ShuShu_PID = ShuShu.getClosedLoopController();

    private final TalonFX CC = new TalonFX(IntakeConstants.CC_ID, "cantivore");
    
    public Algae(){
        SparkMaxConfig ShuShu_Config = new SparkMaxConfig();
        // AbsoluteEncoderConfig ShuShu_Encoder_Config = new AbsoluteEncoderConfig();
        var CC_Config = CC.getConfigurator();

        ShuShu_Config.idleMode(SparkBaseConfig.IdleMode.kBrake);
        ShuShu_Config.encoder.inverted(false);
        ShuShu_Config.encoder.positionConversionFactor(360);
        ShuShu_Config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        ShuShu_Config.closedLoop.pid(0.0, 0.0, 0.0);

        // ShuShu.configure(ShuShu_Config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

        CC.setNeutralMode(NeutralModeValue.Brake);

        ShuShu_Config.inverted(IntakeConstants.ShuShu_Inverted);
        CC.setInverted(IntakeConstants.CC_Inverted);

        // CC_Config.apply(new FeedbackConfigs()
        //         .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor));

        // set maximum acceleration and velocity
        
        // // Sets the mechanism position of the device in mechanism rotations.
        // ShuShuConfig.setPosition(0);
        // CCConfig.setPosition(0);

        // // PIDConfig
        // Slot0Configs PIDConfig = new Slot0Configs();
        // PIDConfig.kP = IntakeConstants.P;
        // PIDConfig.kI = IntakeConstants.I;
        // PIDConfig.kD = IntakeConstants.D;
        // PIDConfig.kV = IntakeConstants.F;
        // ShuShuConfig.apply(PIDConfig);
        // CCConfig.apply(PIDConfig);
    }

    public void ShuShu(){
        ShuShu_PID.setReference(Constants.IntakeConstants.ShuShu_Longest, ControlType.kMAXMotionPositionControl);
    }

    public void BomBom(){
        ShuShu_PID.setReference(Constants.IntakeConstants.ShuShu_Shortest, ControlType.kPosition);
    }

    public void ShuBom(){
        ShuShu_PID.setReference(Constants.IntakeConstants.ShuShu_Middle,ControlType.kPosition);
    }

    public void Shu(){
        ShuShu.set(0.1);
    }

    public void Bom(){
        ShuShu.set(-0.05);
    }

    public void CC(){
        CC.set(0.8);
    }

    public void TT(){
        CC.set(-0.8);
    }

    public void ShuC(){
        ShuShu.set(0.08);
        CC.set(0.8);
    }

    public void BomT(){
        ShuShu.set(-0.08);
        CC.set(-0.8);
    }

    public void Stop(){
        ShuShu.set(0);
        CC.set(0);
    }
}