package frc.robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// 3 is drive
// 4 is steer
public class Swerve {
    
    TalonFX driveKraken;
    CANSparkMax steerSpark;
    CANcoder cancoder;
    double[] offset = {-0.37622, -0.09009, 0.36816, -0.00342};


    public Swerve(int driveID, int steerID, int canCoderID, boolean driveOnTop, boolean steerOnTop) {
        driveKraken = new TalonFX(driveID, "*");
        TalonFXConfiguration talonConfig = new TalonFXConfiguration();

        if (driveOnTop) {
            talonConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        }
        else {
            talonConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        }
        talonConfig.CurrentLimits.StatorCurrentLimit = 45;
        talonConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        driveKraken.getConfigurator().apply(talonConfig);
        



        steerSpark = new CANSparkMax(steerID, MotorType.kBrushless);
        steerSpark.restoreFactoryDefaults();
        steerSpark.setInverted(!steerOnTop);



        cancoder = new CANcoder(canCoderID, "*");
        CANcoderConfiguration configs = new CANcoderConfiguration();
        configs.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        configs.MagnetSensor.MagnetOffset = offset[canCoderID-4];
        configs.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        cancoder.getConfigurator().apply(configs);
    }
    double diffrence = 0;
    double o = 0;
    public void alignWheels(double desiredPosition) {
        double position = cancoder.getPosition().getValueAsDouble();
        if (desiredPosition - position > 0.5) {
            while (desiredPosition - position > 0.5) {
                desiredPosition = desiredPosition - 1;
            }

        } else if (desiredPosition - position < -0.5) {
            while (desiredPosition - position < -0.5) {
                desiredPosition = desiredPosition + 1;
            }
        }
        diffrence = desiredPosition - position;
        o = diffrence * 60;
        steerSpark.setVoltage(o);
    }
    public void spin(double desiredMPS) {

        double radiusMeters = Units.inchesToMeters(2);
        double circumference = 2 * Math.PI * radiusMeters;
        double rotationsPerSecond = desiredMPS / circumference;
        double desiredMotorSpinsPS = rotationsPerSecond * 6.75;
        double desiredMotorSpinsPM = desiredMotorSpinsPS * 60;
        double math1 = desiredMotorSpinsPM / 483.917;
        double math2 = 14.333 / 483.917;
        double volt = math1 + math2;
        driveKraken.setVoltage(volt);
    }
    public void drive(SwerveModuleState desiredState){
        desiredState = SwerveModuleState.optimize(desiredState, Rotation2d.fromRotations(cancoder.getPosition().getValueAsDouble()));
        alignWheels(desiredState.angle.getRotations());
        spin(desiredState.speedMetersPerSecond);
    }
    public void getSpeed() {
        double speed = driveKraken.getVelocity().getValueAsDouble();
        SmartDashboard.putNumber("Rotations Per Second", speed);
    }

    public double getDistance() {
        double driveMotorRotations = driveKraken.getPosition().getValueAsDouble();
        double wheelRotationsPerMotorRotation = (16.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0);
        double driveWheelRotations = driveMotorRotations * wheelRotationsPerMotorRotation;

        double radiusMeters = Units.inchesToMeters(2);
        double circumference = 2 * Math.PI * radiusMeters;
        double distanceMeters = driveWheelRotations * circumference;
        return distanceMeters;
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDistance(), Rotation2d.fromRotations(cancoder.getPosition().getValueAsDouble()));
    }

}