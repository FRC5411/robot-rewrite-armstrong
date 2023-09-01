
package frc.robot.Subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Utils.ConsoleWriter;
import frc.robot.Utils.SparkMaxConfigs;

public class DriveSubsystem extends SubsystemBase {

  private CANSparkMax m_leftFrontMotor;
  private CANSparkMax m_leftBackMotor;
  private CANSparkMax m_rightFrontMotor;
  private CANSparkMax m_rightBackMotor;

  private RelativeEncoder m_leftFrontEncoder;
  private RelativeEncoder m_leftBackEncoder;
  private RelativeEncoder m_rightFrontEncoder;
  private RelativeEncoder m_rightBackEncoder;

  private MotorControllerGroup m_leftMotors;
  private MotorControllerGroup m_rightMotors;

  private DifferentialDrive m_robotDrive;

  private AHRS m_navX;

  private DifferentialDrivePoseEstimator m_robotOdometry;

  private Field2d m_field;

  public DriveSubsystem() {
    m_leftFrontMotor = SparkMaxConfigs.NEO(DriveConstants.kLeftFront, 
      false);
    m_leftBackMotor = SparkMaxConfigs.NEO(DriveConstants.kLeftBack, 
      false);
    m_rightFrontMotor = SparkMaxConfigs.NEO(DriveConstants.kRightFront, 
      true);
    m_rightBackMotor = SparkMaxConfigs.NEO(DriveConstants.kRightBack, 
      true);

    m_leftFrontEncoder = SparkMaxConfigs.RelativeEncoder(m_leftFrontMotor, 
      DriveConstants.kLinearDistanceConversion);
    m_leftBackEncoder = SparkMaxConfigs.RelativeEncoder(m_leftBackMotor, 
      DriveConstants.kLinearDistanceConversion);
    m_rightFrontEncoder = SparkMaxConfigs.RelativeEncoder(m_rightFrontMotor, 
      DriveConstants.kLinearDistanceConversion);
    m_rightBackEncoder = SparkMaxConfigs.RelativeEncoder(m_rightBackMotor, 
      DriveConstants.kLinearDistanceConversion);

    m_leftMotors = new MotorControllerGroup(m_leftFrontMotor, m_leftBackMotor);
    m_rightMotors = new MotorControllerGroup(m_rightFrontMotor, m_rightBackMotor);

    m_robotDrive = new DifferentialDrive(m_leftMotors, m_rightMotors);

    m_navX = new AHRS(SPI.Port.kMXP);
    m_navX.calibrate();
    m_navX.reset();

    m_robotOdometry = new DifferentialDrivePoseEstimator(DriveConstants.kTankKinematics, 
      getHeading(), 
      getPosition(m_leftFrontEncoder, m_leftFrontEncoder.getInverted()), 
      getPosition(m_rightFrontEncoder, m_rightFrontEncoder.getInverted()), 
      new Pose2d());

    m_field = new Field2d();
    SmartDashboard.putData(m_field);

    resetOdometry(getPose());
  }

  /* ---------------------------------------------------------------------------------------- */

  public void resetOdometry(Pose2d pose) {
    resetEncoders();

    m_robotOdometry.resetPosition(getHeading(), 
      getPosition(m_leftFrontEncoder, m_leftFrontEncoder.getInverted()),
      getPosition(m_rightFrontEncoder, m_rightFrontEncoder.getInverted()),
      pose);
  }

  public void resetGyro() {
    m_navX.setAngleAdjustment(getHeading().getDegrees());
  }

  public void resetEncoders() {
    m_leftFrontEncoder.setPosition(0.0);
    m_leftBackEncoder.setPosition(0.0);
    m_rightFrontEncoder.setPosition(0.0);
    m_rightBackEncoder.setPosition(0.0);
  }

  /* ---------------------------------------------------------------------------------------- */

  public void setTankDriveVolts(double leftVolts, double rightVolts) {
    m_leftMotors.setVoltage(leftVolts);
    m_rightMotors.setVoltage(rightVolts);

    m_robotDrive.feed();
  }

  public void setField() {
    m_field.setRobotPose(getPose());
  }

  public void updateOdometry() {
    m_robotOdometry.update(getHeading(), 
      getPosition(m_leftFrontEncoder, m_leftFrontEncoder.getInverted()), 
      getPosition(m_rightFrontEncoder, m_rightFrontEncoder.getInverted()));
  }

  /* ---------------------------------------------------------------------------------------- */

  public void arcadeDrive(double ySpeed, double xRotation, boolean squareInputs) {
    m_robotDrive.arcadeDrive(ySpeed, xRotation, squareInputs);
    m_robotDrive.feed();
  }

  /* ---------------------------------------------------------------------------------------- */

  public double getPosition(RelativeEncoder encoder, boolean inverted) {
    return inverted ? -encoder.getPosition() : encoder.getPosition();
  }

  public double getVelocity(RelativeEncoder encoder, boolean inverted) {
    return inverted ? -encoder.getVelocity() : encoder.getVelocity();
  }

  public double getTemperature(CANSparkMax motor) {
    return motor.getMotorTemperature();
  }

  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(m_navX.getYaw());
  }

  public Pose2d getPose() {
    return m_robotOdometry.getEstimatedPosition();
  }

  /* ---------------------------------------------------------------------------------------- */

  @Override
  public void periodic() {
    SparkMaxConfigs.configureTelemetry(m_leftFrontMotor, m_leftFrontEncoder, "Left Front");
    SparkMaxConfigs.configureTelemetry(m_leftBackMotor, m_leftBackEncoder, "Left Back");
    SparkMaxConfigs.configureTelemetry(m_rightFrontMotor, m_rightFrontEncoder, "Right Front");
    SparkMaxConfigs.configureTelemetry(m_rightBackMotor, m_rightBackEncoder, "Right Back");

    if (getTemperature(m_leftBackMotor) > 50) { ConsoleWriter.printError("leftBackMotor is above operating temperature: " + 50 + "C", getName());}

    updateOdometry();
    setField();
  }
}
