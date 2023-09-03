
package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Utils.Configs;
import frc.robot.Utils.ConsoleWriter;

public class IntakeSubsystem extends SubsystemBase {
  private CANSparkMax m_intakeMotor;

  private RelativeEncoder m_intakeRelativeEncoder;

  private boolean m_isSafeToRunIntake;

  public IntakeSubsystem() {
    m_intakeMotor = Configs.NEO(IntakeConstants.kIntakeMotor, 
      false, 
      IntakeConstants.kIntakeMotorCurrentLimit, 
      IdleMode.kBrake);

    m_intakeRelativeEncoder = m_intakeMotor.getEncoder();

    m_isSafeToRunIntake = false;
  }

  /* ---------------------------------------------------------------------------------------- */

  public void setIntake(double speed) {
    if (m_isSafeToRunIntake) { m_intakeMotor.set(speed); }
    else { ConsoleWriter.printError("Cannot run intake", getName()); }
  }

  /* ---------------------------------------------------------------------------------------- */

  public boolean isSafeToRunIntake(int intakeCurrentLimit) {
    LinearFilter filter = LinearFilter.movingAverage(25);
    double filterCalculation = filter.calculate(getIntakeOutputCurrent());

    if (filterCalculation > intakeCurrentLimit) { return false; }
    
    return true;
  }

  /* ---------------------------------------------------------------------------------------- */

  public double getIntakeOutputCurrent() {
    return m_intakeMotor.getOutputCurrent();
  }

  /* ---------------------------------------------------------------------------------------- */
  
  @Override
  public void periodic() {
    Configs.configureTelemetry(m_intakeMotor, m_intakeRelativeEncoder, "Intake Motor");

    m_isSafeToRunIntake = isSafeToRunIntake(IntakeConstants.kIntakeMotorCurrentLimit);
  }
}
