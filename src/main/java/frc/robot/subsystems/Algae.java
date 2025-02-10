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
    // private final TalonFX CC = new TalonFX(IntakeConstants.CC_ID, "cantivore");
    private final TalonFX CC = new TalonFX(IntakeConstants.CC_ID, "rio");

    private final SparkMaxConfig Intake_ctrl_Config;

    private final AbsoluteEncoder Intake_ctrl_Encoder = new AbsoluteEncoder(){
        @Override
        public double getPosition() {
            return 0;
        }
        public double getVelocity() {
            return 0;
        }
    };

    //  private final ArmFeedforward Shu_Feedforward = new ArmFeedforward(
    //     IntakeConstants.kSVolts, IntakeConstants.kGVolts,
    //     IntakeConstants.kVVoltSecondPerRad, IntakeConstants.kAVoltSecondSquaredPerRad
    // );

    private final SparkClosedLoopController Intake_ctrl_PID = Intake_ctrl.getClosedLoopController();
        
    public Algae(){
        Intake_ctrl_Config = new SparkMaxConfig();
        AbsoluteEncoderConfig Intake_ctrl_Encoder_Config = new AbsoluteEncoderConfig();
        var Roller_Config = Roller.getConfigurator();

        // ShuShu Config
        Intake_ctrl_Config.idleMode(SparkBaseConfig.IdleMode.kBrake);
        Intake_ctrl_Config.inverted(IntakeConstants.Intake_ctrl_Inverted);
        Intake_ctrl_Config.encoder.positionConversionFactor(1);
        Intake_ctrl_Config.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
        
        Intake_ctrl_Config.closedLoop.pid(0.025, 0.0, 0.0, ClosedLoopSlot.kSlot2);
        Intake_ctrl_Config.closedLoop.pid(1, 0.0, 0.0, ClosedLoopSlot.kSlot1);  

        Intake_ctrl.configure(Intake_ctrl_Config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

        // ShuShu Encoder Config
        Intake_ctrl_Encoder_Config.inverted(Constants.IntakeConstants.Intake_ctrl_Inverted);
        Intake_ctrl_Encoder_Config.positionConversionFactor(360);

        CC.setNeutralMode(NeutralModeValue.Brake);
        CC.setInverted(IntakeConstants.CC_Inverted);

        // CC_Config.apply(new FeedbackConfigs()
        //         .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor));

        // set maximum acceleration and velocity
        
        // // Sets the mechanism position of the device in mechanism rotations.
        // ShuShuConfig.setPosition(0);
        // CCConfig.setPosition(0);

        // PIDConfig
    }

    public double getPosition(){
        return Intake_ctrl_Encoder.getPosition();
    }

    public double getFurtherest(){
        return (1.3 - Intake_ctrl_Encoder.getPosition());
    }
 
    public double getShortest(){
        return (1.3 - Intake_ctrl_Encoder.getPosition() - 1.3);
    }
    
    public void Intake_out(){
        // final SparkClosedLoopController ShuShu_PID = ShuShu.getClosedLoopController();
        // ShuShu_Config.closedLoop.pid(0.025, 0.0, 0.0);
        Intake_ctrl_PID.setReference(getFurtherest() * 2 * Math.PI, ControlType.kPosition, ClosedLoopSlot.kSlot2); 
   }

    public void Intake_back(){
        // final SparkClosedLoopController ShuShu_PID = ShuShu.getClosedLoopController();
        // ShuShu_Config.closedLoop.pid(0.05, 0.0, 0.0);
        Intake_ctrl_PID.setReference(getShortest() * 2 * Math.PI, ControlType.kPosition, ClosedLoopSlot.kSlot1);

    }

    public void Intake_hold(){
        // ShuShu_PID.setReference(Constants.IntakeConstants.ShuShu_Middle,ControlType.kPosition);
        // ShuShu_PID.setReference(getFurtherest() * 2 * Math.PI, ControlType.kPosition);

    }

    public void step_out(){
        Intake_ctrl.set(0.1);
    }

    public void step_in(){
        Intake_ctrl.set(-0.2);
    }

    public void suck(){
        Roller.set(0.4);
    }

    public void shot(){
        Roller.set(-0.6);
    }

    // public void ShuC(){
    //     Intake_ctrl.set(0.08);
    //     Roller.set(0.8);
    // }

    // public void BomT(){
    //     Intake_ctrl.set(-0.08);
    //     Roller.set(-0.8);
    // }

    public void Stop(){
        Intake_ctrl.set(0);
        Roller.set(0);
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("position", Intake_ctrl_Encoder.getPosition());
    }
}
