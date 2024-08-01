package frc.robot;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.util.Units;
public class Arm {
    private CANSparkMax leftMotor;
    private CANSparkMax rightMotor;
    private CANcoder cancoder;
    double armLockMath;
    
    public Arm() {
        final  int leftMotorID = 7;
        final int rightMotorID = 8;
        final int leftArmCANcoderID = 9;
        final  int rightArmCANcoderID = 8;
        final double rightArmCANcoderOffset = -0.431;
        final double leftArmCANcoderOffset = 0.100;
        cancoder = new CANcoder(leftArmCANcoderID, "*");
        CANcoderConfiguration configs = new CANcoderConfiguration();
        configs.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        configs.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        configs.MagnetSensor.MagnetOffset = leftArmCANcoderOffset;
        cancoder.getConfigurator().apply(configs);
        leftMotor = new CANSparkMax(leftMotorID, MotorType.kBrushless);
        rightMotor = new CANSparkMax(rightMotorID, MotorType.kBrushless);
        int ampLimit = 30;
        rightMotor.restoreFactoryDefaults();
        rightMotor.setSmartCurrentLimit(ampLimit);
        rightMotor.setInverted(false);
        rightMotor.setIdleMode(IdleMode.kBrake);
        rightMotor.burnFlash();
        leftMotor.restoreFactoryDefaults();
        leftMotor.setSmartCurrentLimit(ampLimit);
        leftMotor.setInverted(true);
        leftMotor.setIdleMode(IdleMode.kBrake);
        leftMotor.burnFlash();
    }
    public void moveArm(double input) {
        if (cancoder.getPosition().getValueAsDouble() < -0.05 && input < 0) {
            leftMotor.setVoltage(0);
            rightMotor.setVoltage(0);
        } else {
            leftMotor.setVoltage(input);
            rightMotor.setVoltage(input);
        }
    }
    public void stopMotor() {
        leftMotor.setVoltage(0);
        rightMotor.setVoltage(0);
        System.out.println(cancoder.getPosition().getValueAsDouble());
    }
    public void lockArm() {
         if (cancoder.getPosition().getValueAsDouble() < -0.05) {
         double voltsToNotLock = 0;
         stopMotor();
        } else {
            double armAngleRotations = cancoder.getAbsolutePosition().getValueAsDouble();
            double armAngleRadians = Units.rotationsToRadians(armAngleRotations);
            double voltsToLock = Math.cos(armAngleRadians) * 0.32;
            moveArm(voltsToLock);
        }
    }
    
}
