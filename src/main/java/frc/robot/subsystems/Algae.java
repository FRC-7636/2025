package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

// NEO Motor - for 
// Karken Motor - for 
public class Algae extends SubsystemBase {
    private final SparkMax Intake_Ctrl = new SparkMax(IntakeConstants.Intake_ctrl_ID, MotorType.kBrushless);
    private final TalonFX Roller = new TalonFX(IntakeConstants.Roller_ID, "rio");

    private final SparkMaxConfig Intake_Ctrl_Config;
    private final AbsoluteEncoder Intake_Ctrl_Encoder = new AbsoluteEncoder(){
        @Override
        public double getPosition() {
            return 0;
        }
        public double getVelocity() {
            return 0;
        }
    };

    private final SparkClosedLoopController Intake_Ctrl_PID = Intake_Ctrl.getClosedLoopController();
        
    public Algae(){
        Intake_Ctrl_Config = new SparkMaxConfig();
        AbsoluteEncoderConfig Intake_Ctrl_Encoder_Config = new AbsoluteEncoderConfig();
        var Roller_Config = Roller.getConfigurator();

        // Intake_ctrl Config
        Intake_Ctrl_Config.idleMode(SparkBaseConfig.IdleMode.kBrake);
        Intake_Ctrl_Config.inverted(IntakeConstants.Intake_ctrl_Inverted);
        Intake_Ctrl_Config.encoder.positionConversionFactor(1);
        Intake_Ctrl_Config.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
        Intake_Ctrl_Config.closedLoop.pid(0.025, 0.0, 0.0, ClosedLoopSlot.kSlot2);
        Intake_Ctrl_Config.closedLoop.pid(0.8, 0.0, 0.0, ClosedLoopSlot.kSlot1);  

        Intake_Ctrl.configure(Intake_Ctrl_Config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

        // ShuShu Encoder Config
        Intake_Ctrl_Encoder_Config.inverted(Constants.IntakeConstants.Intake_ctrl_Inverted);
        Intake_Ctrl_Encoder_Config.positionConversionFactor(360);

        Roller.setNeutralMode(NeutralModeValue.Brake);
        Roller.setInverted(IntakeConstants.Roller_Inverted);
    }

    public double getPosition(){
        return Intake_Ctrl_Encoder.getPosition();
    }

    public double getFurtherest(){
        return (1.3 - Intake_Ctrl_Encoder.getPosition());
    }
 
    public double getShortest(){
        return (1.3 - Intake_Ctrl_Encoder.getPosition() - 1.3);
    }
    
    public void Intake_out(){
        Intake_Ctrl_PID.setReference(getFurtherest() * 2 * Math.PI, ControlType.kPosition, ClosedLoopSlot.kSlot2); 
        Timer.delay(2);
        Intake_Ctrl.set(0);
   }

    public void Intake_back(){
        Intake_Ctrl_PID.setReference(getShortest() * 2 * Math.PI, ControlType.kPosition, ClosedLoopSlot.kSlot1);
        Timer.delay(2);
        Intake_Ctrl.set(0);
    }

    // public void Intake_hold(){
    //     Intake_Ctrl_PID.setReference(getFurtherest() * 2 * Math.PI, ControlType.kPosition, ClosedLoopSlot.kSlot1);
    // }

    public void step_out(){
        Intake_Ctrl.set(0.1);
    }

    public void step_in(){
        Intake_Ctrl.set(-0.2);
    }

    public void suck(){
        Roller.set(-0.3);
    }

    public void shot(){
        Roller.set(0.2);
    }

    public void Stop(){
        Intake_Ctrl.set(0);
        Roller.set(0);
    }
}
