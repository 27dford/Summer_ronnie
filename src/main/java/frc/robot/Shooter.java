package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter {
    double rightSpeed;
    double leftSpeed;
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
    public boolean setSpeed(double desiredSpeedMPSL, double desiredSpeedMPSR){
        double rotationsPerSecond = flyKrakenLeft.getVelocity().getValueAsDouble();
        double radiansPerSecond = Units.rotationsToRadians(rotationsPerSecond);
        double metersPerSecond1 = Units.inchesToMeters(2) * radiansPerSecond;
        double mpsChange = desiredSpeedMPSL - metersPerSecond1;

        double extraVolts = mpsChange * 0.6;
        double baseVolts = desiredSpeedMPSL * 0.4047;
        flyKrakenLeft.setVoltage(baseVolts + extraVolts);

        rotationsPerSecond = flyKrakenRight.getVelocity().getValueAsDouble();
        radiansPerSecond = Units.rotationsToRadians(rotationsPerSecond);
        double metersPerSecond2 = Units.inchesToMeters(2) * radiansPerSecond;
         mpsChange = desiredSpeedMPSR - metersPerSecond2;
         extraVolts = mpsChange * 0.6;
         baseVolts = desiredSpeedMPSR * 0.4047;
        flyKrakenRight.setVoltage(baseVolts + extraVolts);
        if (((metersPerSecond1 < 26) && (metersPerSecond1 > 24)) && ((metersPerSecond2 < 21) && (metersPerSecond2 > 19))) {
            return true;
        } else {
            SmartDashboard.putNumber("Right flywheel measured", metersPerSecond2);
            SmartDashboard.putNumber("Right flywheel desired", desiredSpeedMPSR);
            SmartDashboard.putNumber("Left flywheel measured", metersPerSecond1);
            SmartDashboard.putNumber("Left flywheel desired", desiredSpeedMPSL);
            return false;
        }

    }
    public void shoot() {

        flyKrakenLeft.setVoltage(10);
        flyKrakenRight.setVoltage(10);

    }
    public void notShoot() {
        flyKrakenLeft.setVoltage(0);
        flyKrakenRight.setVoltage(0);
    }
    public void moveArm(double input) {

    }
}
