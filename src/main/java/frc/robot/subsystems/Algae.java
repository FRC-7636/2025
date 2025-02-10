package frc.robot.subsystems;

import java.time.Period;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
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

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

import edu.wpi.first.math.controller.ArmFeedforward;

// NEO Motor - for 
// Karken Motor - for 
public class Algae extends SubsystemBase {
    private final SparkMax ShuShu = new SparkMax(IntakeConstants.ShuShu_ID, MotorType.kBrushless);
    private final TalonFX CC = new TalonFX(IntakeConstants.CC_ID, "rio");

    private final SparkMaxConfig ShuShu_Config;

    private final AbsoluteEncoder ShuShuEncoder = new AbsoluteEncoder(){
        @Override
        public double getPosition() {
            return 0;
        }
        public double getVelocity() {
            return 0;
        }
    };

     private final ArmFeedforward Shu_Feedforward = new ArmFeedforward(
        IntakeConstants.kSVolts, IntakeConstants.kGVolts,
        IntakeConstants.kVVoltSecondPerRad, IntakeConstants.kAVoltSecondSquaredPerRad
    );

    private final SparkClosedLoopController ShuShu_PID = ShuShu.getClosedLoopController();
        
    public Algae(){
        ShuShu_Config = new SparkMaxConfig();
        AbsoluteEncoderConfig ShuShu_Encoder_Config = new AbsoluteEncoderConfig();
        var CC_Config = CC.getConfigurator();

        // ShuShu Config
        ShuShu_Config.idleMode(SparkBaseConfig.IdleMode.kBrake);
        ShuShu_Config.inverted(IntakeConstants.Shu_Inverted);
        ShuShu_Config.encoder.positionConversionFactor(1);
        ShuShu_Config.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
        
        ShuShu_Config.closedLoop.pid(0.025, 0.0, 0.0, ClosedLoopSlot.kSlot2);
        ShuShu_Config.closedLoop.pid(1, 0.0, 0.0, ClosedLoopSlot.kSlot1);  

        ShuShu.configure(ShuShu_Config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

        // ShuShu Encoder Config
        ShuShu_Encoder_Config.inverted(Constants.IntakeConstants.Shu_Inverted);
        ShuShu_Encoder_Config.positionConversionFactor(360);

        CC.setNeutralMode(NeutralModeValue.Brake);
        CC.setInverted(IntakeConstants.CC_Inverted);
    }

    public double getPosition(){
        return ShuShuEncoder.getPosition();
    }

    public double getFurtherest(){
        return (1.3 - ShuShuEncoder.getPosition());
    }
 
    public double getShortest(){
        return (1.3 - ShuShuEncoder.getPosition() - 1.3);
    }
    
    public void ShuShu(){
        // final SparkClosedLoopController ShuShu_PID = ShuShu.getClosedLoopController();
        // ShuShu_Config.closedLoop.pid(0.025, 0.0, 0.0);
        ShuShu_PID.setReference(getFurtherest() * 2 * Math.PI, ControlType.kPosition, ClosedLoopSlot.kSlot2); 
   }

    public void BomBom(){
        // final SparkClosedLoopController ShuShu_PID = ShuShu.getClosedLoopController();
        // ShuShu_Config.closedLoop.pid(0.05, 0.0, 0.0);
        ShuShu_PID.setReference(getShortest() * 2 * Math.PI, ControlType.kPosition, ClosedLoopSlot.kSlot1);

    }

    public void ShuBom(){
        // ShuShu_PID.setReference(Constants.IntakeConstants.ShuShu_Middle,ControlType.kPosition);
        // ShuShu_PID.setReference(getFurtherest() * 2 * Math.PI, ControlType.kPosition);

    }

    public void Shu(){
        ShuShu.set(0.1);
    }

    public void Bom(){
        ShuShu.set(-0.2);
    }

    public void CC(){
        CC.set(0.4);
    }

    public void TT(){
        CC.set(-0.6);
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

    @Override
    public void periodic(){
        SmartDashboard.putNumber("position", ShuShuEncoder.getPosition());
    }
}
