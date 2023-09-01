
package frc.robot.Utils;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.RelativeEncoder;

public class Configs {

    public static CANSparkMax NEO(int deviceID, boolean inverted) {
        CANSparkMax motor = new CANSparkMax(deviceID, MotorType.kBrushless);
        motor.clearFaults();
        motor.setSmartCurrentLimit(40);
        motor.setSecondaryCurrentLimit(40);
        motor.setIdleMode(IdleMode.kBrake);
        motor.setInverted(inverted);
        return motor;
    }

    public static CANSparkMax NEO(int deviceID, boolean inverted, int smartCurrentLimit, IdleMode idleMode) {
        CANSparkMax motor = new CANSparkMax(deviceID, MotorType.kBrushless);
        motor.clearFaults();
        motor.setSmartCurrentLimit(smartCurrentLimit);
        motor.setSecondaryCurrentLimit(40);
        motor.setIdleMode(idleMode);
        motor.setInverted(inverted);
        return motor;
    }

    public static RelativeEncoder RelativeEncoder(CANSparkMax motor, double CF) {
        RelativeEncoder encoder = motor.getEncoder();
        encoder.setPositionConversionFactor(CF);
        encoder.setVelocityConversionFactor(CF / 60);
        return encoder;
    }

    public static Encoder BoreEncoder(int channelA, int channelB, double CF, boolean isInverted) {
        Encoder encoder = new Encoder(channelA, channelB);
        encoder.setReverseDirection(isInverted);
        encoder.setDistancePerPulse(CF);
        return encoder;
    }

    public static void configureTelemetry(CANSparkMax motor, RelativeEncoder encoder, String key) {
        SmartDashboard.putNumber(key + "/Position", encoder.getPosition());
        SmartDashboard.putNumber(key + "/Velocity", encoder.getVelocity());
        SmartDashboard.putNumber(key + "/TemperatureC", motor.getMotorTemperature());
        SmartDashboard.putNumber(key + "/OutputCurrent", motor.getOutputCurrent());
        SmartDashboard.putNumber(key + "/Bus Voltage", motor.getBusVoltage());
    }

    public static void configureTelemetry(CANSparkMax motor, Encoder encoder, double CF, String key) {
        SmartDashboard.putNumber(key + "/Position", encoder.getDistance() / CF);
        SmartDashboard.putNumber(key + "/Velocity", encoder.getRate() / CF);
        SmartDashboard.putNumber(key + "/TemperatureC", motor.getMotorTemperature());
        SmartDashboard.putNumber(key + "/OutputCurrent", motor.getOutputCurrent());
        SmartDashboard.putNumber(key + "/Bus Voltage", motor.getBusVoltage());
    }
}
