
package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.Configs;
import frc.robot.Utils.ConsoleWriter;

public class ArmSubsystem extends SubsystemBase {

  private CANSparkMax m_armMotor;
  private CANSparkMax m_intakeMotor;

  private RelativeEncoder m_armRelativeEncoder;
  private RelativeEncoder m_intakeRelativeEncoder;

  private Encoder m_armEncoder;

  private boolean m_isSafeToRunIntake;

  public ArmSubsystem() {
    m_armMotor = Configs.NEO(21, false, 60, IdleMode.kBrake);
    m_intakeMotor = Configs.NEO(22, false, 50, IdleMode.kBrake);

    m_armRelativeEncoder = m_armMotor.getEncoder();
    m_intakeRelativeEncoder = m_intakeMotor.getEncoder();

    m_armEncoder = new Encoder(0, 1);

    m_isSafeToRunIntake = false;
  }

  private double boreConversion(double measurement) {
    return measurement / 22.755;
  }

  /* ---------------------------------------------------------------------------------------- */

  public void setArm(double speed) {
    if (isSafeToMoveArm(speed)) { m_armMotor.set(speed); }
    else { ConsoleWriter.printError("Cannot move arm", getName()); }
  }

  public void setIntake(double speed) {
    if (m_isSafeToRunIntake) { m_intakeMotor.set(speed); }
    else { ConsoleWriter.printError("Cannot run intake", getName()); }
  }

  /* ---------------------------------------------------------------------------------------- */

  public double getArmPosition() {
    return boreConversion(m_armEncoder.getDistance());
  }

  public double getArmVelocity() {
    return m_armRelativeEncoder.getVelocity() * 2 * Math.PI;
  }

  public double getArmEncoderVelocity() {
    return boreConversion(m_armEncoder.getRate());
  }

  public double getArmOutputCurrent() {
    return m_armMotor.getOutputCurrent();
  }

  public double getIntakeOutputCurrent() {
    return m_intakeMotor.getOutputCurrent();
  }

  public boolean isSafeToMoveArm(double desiredSpeed) {
    if ( (getArmPosition() > 263 && desiredSpeed > 0) || 
        (getArmPosition() < 3 && desiredSpeed < 0)) {

      return false;
    }
    return true;
  }

  public boolean isSafeToRunIntake(int intakeCurrentLimit) {
    LinearFilter filter = LinearFilter.movingAverage(25);
    double filterCalculation = filter.calculate(getIntakeOutputCurrent());

    if (filterCalculation > intakeCurrentLimit) {
      return false;
    }
    return true;
  }

  /* ---------------------------------------------------------------------------------------- */

  @Override
  public void periodic() {
    Configs.configureTelemetry(m_armMotor, m_armEncoder, 22.755,"Arm Motor");
    SmartDashboard.putNumber("Arm Motor/Arm Speed", m_armMotor.get());

    Configs.configureTelemetry(m_intakeMotor, m_intakeRelativeEncoder, "Intake Motor");

    m_isSafeToRunIntake = isSafeToRunIntake(50);
  }
}
