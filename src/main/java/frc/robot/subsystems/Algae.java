package frc.robot.subsystems;

import java.util.zip.CRC32C;

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

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.AlgaeConstants;

// 2 Karken Motor - for 
public class Algae extends SubsystemBase {
    private final TalonFX Algae_Ctrl = new TalonFX(AlgaeConstants.Algae_Ctrl_ID, "mech"); 
    // private final TalonFX Algae_Ctrl = new TalonFX(AlgaeConstants.Algae_Ctrl_ID, "cantivore");
    private final TalonFX Algae_Roller = new TalonFX(AlgaeConstants.Algae_Roller_ID, "mech");

    private final SparkMaxConfig Intake_Ctrl_Config; //
    private final AbsoluteEncoder Intake_Ctrl_Encoder = new AbsoluteEncoder(){
        @Override
        public double getPosition() {
            return 0;
        }
        public double getVelocity() {
            return 0;
        }
    };

    // private final SparkClosedLoopController Intake_Ctrl_PID = Algae_Ctrl.getClosedLoopController();
        
    public Algae(){
        Intake_Ctrl_Config = new SparkMaxConfig(); //
        AbsoluteEncoderConfig Intake_Ctrl_Encoder_Config = new AbsoluteEncoderConfig();
        var Roller_Config = Algae_Roller.getConfigurator();

        // Algae_ctrl Config
        Intake_Ctrl_Config.idleMode(SparkBaseConfig.IdleMode.kBrake);
        Intake_Ctrl_Config.inverted(AlgaeConstants.Algae_ctrl_Inverted);
        Intake_Ctrl_Config.encoder.positionConversionFactor(1);
        Intake_Ctrl_Config.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
        Intake_Ctrl_Config.closedLoop.pid(0.025, 0.0, 0.0, ClosedLoopSlot.kSlot2);
        Intake_Ctrl_Config.closedLoop.pid(2560, 0.0, 0.0, ClosedLoopSlot.kSlot1);  

        // Algae_Ctrl.configure(Intake_Ctrl_Config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

        // ShuShu Encoder Config
        Intake_Ctrl_Encoder_Config.inverted(Constants.AlgaeConstants.Algae_ctrl_Inverted);
        Intake_Ctrl_Encoder_Config.positionConversionFactor(360);

        Algae_Roller.setNeutralMode(NeutralModeValue.Brake);
        Algae_Roller.setInverted(AlgaeConstants.Algae_Roller_Inverted);
    }

    public double getPosition(){
        return Intake_Ctrl_Encoder.getPosition();
    }

    public double getFurtherest(){
        return (1.5 - Intake_Ctrl_Encoder.getPosition());
    }
 
    public double getShortest(){
        return (1.5 - Intake_Ctrl_Encoder.getPosition() - 1.5);
    }
    
    public void Intake_out(){
        // Intake_Ctrl_PID.setReference(getFurtherest() * 2 * Math.PI, ControlType.kPosition, ClosedLoopSlot.kSlot2); 
        Algae_Ctrl.set(0.1);
        // Timer.delay(2);
        // Algae_Ctrl.set(0);
   }

    public void Intake_back(){
        // Intake_Ctrl_PID.setReference(getShortest() * 2 * Math.PI, ControlType.kPosition, ClosedLoopSlot.kSlot1);
        Algae_Ctrl.set(-0.15);
        // Timer.delay(2);
        // Algae_Ctrl.set(0);
    }

    public void Intake_hold(){
    //     Intake_Ctrl_PID.setReference(getFurtherest() * 2 * Math.PI, ControlType.kPosition, ClosedLoopSlot.kSlot1);
    }

    public void step_out(){
        Algae_Ctrl.set(0.1);
    }

    public void step_in(){
        Algae_Ctrl.set(-0.2);
    }

    public void suck(){
        Algae_Roller.set(0.3);
    }

    public void shoot(){
        Algae_Roller.set(-0.35);
    }

    public void Stop(){
        Algae_Ctrl.set(0);
        Algae_Roller.set(0);
    }

    public void Algae_Zero(){
        Commands.runOnce( () -> Algae_Ctrl.set(0.15), null).alongWith(new WaitCommand(0.2).andThen( () ->Algae_Ctrl.set(0)));
        Algae_Ctrl.set(0);
        Algae_Roller.set(0);
    }

    @Override 
    public void periodic(){
        SmartDashboard.putNumber("Al_Pos", getPosition());
    }
}
