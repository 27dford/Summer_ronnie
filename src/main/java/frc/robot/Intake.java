package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;

public class Intake {
    private DigitalInput intakeProximitySwitchLeft;
    private DigitalInput intakeProximitySwitchRight;
    private DigitalInput intakeProximitySwitchLeftBottom;
    private DigitalInput intakeProximitySwitchRightBottom;
    private CANSparkMax frontIntakeMotor;
    TalonFX intakeKraken;
    /**
     * Motor object that controls the two axles on the back of the intake.
     * A positive voltage spins the axles to suck a note into the robot.
     */
    private CANSparkMax backIntakeMotor;
    // back motor is the main one
    public final static int frontIntakeMotorID = 4;
    public final static int backIntakeMotorID = 3;
    public static final int intakeProximitySwitchIDLeftBottom = 2;
    public static final int intakeProximitySwitchIDRightBottom = 0;
    public static final int indexerProximitySwitchIDLeft = 3;
    public static final int indexerProximitySwitchIDRight = 1;
    
    public Intake() {
        intakeProximitySwitchLeftBottom = new DigitalInput(intakeProximitySwitchIDLeftBottom);
        intakeProximitySwitchRightBottom = new DigitalInput(intakeProximitySwitchIDRightBottom);
        intakeProximitySwitchLeft = new DigitalInput(indexerProximitySwitchIDLeft);
        intakeProximitySwitchRight = new DigitalInput(indexerProximitySwitchIDRight);
        frontIntakeMotor = new CANSparkMax(frontIntakeMotorID, MotorType.kBrushless);
        backIntakeMotor = new CANSparkMax(backIntakeMotorID, MotorType.kBrushless);
        intakeKraken = new TalonFX(4, "*");
        TalonFXConfiguration talonConfig = new TalonFXConfiguration();
        talonConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        intakeKraken.getConfigurator().apply(talonConfig);
    }
    public void getRing(){
        if (!intakeProximitySwitchLeft.get() || !intakeProximitySwitchRight.get()) {
            frontIntakeMotor.setVoltage(0);
            backIntakeMotor.setVoltage(0);
            intakeKraken.setVoltage(0);
        } else {
            frontIntakeMotor.setVoltage(8);
            backIntakeMotor.setVoltage(8);
            intakeKraken.setVoltage(8);
        }
    }
    public void stopMotor() {
        frontIntakeMotor.setVoltage(0);
        backIntakeMotor.setVoltage(0);
        intakeKraken.setVoltage(0);
    }
    public boolean hasRing() {
        if (!intakeProximitySwitchLeftBottom.get() || !intakeProximitySwitchRightBottom.get()) {
            return true;
        } else {
            return false;
        }
    }



}