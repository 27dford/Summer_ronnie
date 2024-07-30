package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

public class Shooter {
    TalonFX flyKrakenLeft;
    TalonFX flyKrakenRight;
    public Shooter() {
        flyKrakenLeft = new TalonFX(6, "*");
        TalonFXConfiguration talonConfig = new TalonFXConfiguration();
        talonConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        flyKrakenLeft.getConfigurator().apply(talonConfig);
        flyKrakenRight = new TalonFX(5, "*");
        talonConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        flyKrakenRight.getConfigurator().apply(talonConfig);
    }
    public void shoot() {
        flyKrakenLeft.setVoltage(5);
        flyKrakenRight.setVoltage(5);

    }
    public void notShoot() {
        flyKrakenLeft.setVoltage(0);
        flyKrakenRight.setVoltage(0);
    }
}
