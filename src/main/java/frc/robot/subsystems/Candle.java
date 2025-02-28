package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.StrobeAnimation;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Candle extends SubsystemBase{
    private final CANdle candle = new CANdle(10, "mech");
    private final RainbowAnimation rainbowAnimation = new RainbowAnimation(100, 1, 167);

    public Candle() {
        candle.configFactoryDefault();

        candle.clearAnimation(0);
        candle.setLEDs(255, 255, 255);
    }

    private void rainbow(){
        candle.animate(rainbowAnimation);
    }
    @Override
    public void periodic() {
        rainbow();
    }
}
