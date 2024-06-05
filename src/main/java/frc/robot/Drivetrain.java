// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants.DriveTrainConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Represents a mecanum drive style drivetrain. */
public class Drivetrain {

  private final CANSparkMax m_frontLeftMotor = new CANSparkMax(1, MotorType.kBrushless);
  private final CANSparkMax m_frontRightMotor = new CANSparkMax(2, MotorType.kBrushless);
  private final CANSparkMax m_backLeftMotor = new CANSparkMax(3, MotorType.kBrushless);
  private final CANSparkMax m_backRightMotor = new CANSparkMax(4, MotorType.kBrushless);

  private final RelativeEncoder m_frontLeftEncoder = m_frontLeftMotor.getEncoder();
  private final RelativeEncoder m_frontRightEncoder = m_frontRightMotor.getEncoder();
  private final RelativeEncoder m_backLeftEncoder = m_backLeftMotor.getEncoder();
  private final RelativeEncoder m_backRightEncoder = m_backRightMotor.getEncoder();

  private MecanumDrive m_robotDrive = new MecanumDrive(
      m_frontLeftMotor::set, m_backLeftMotor::set, m_frontRightMotor::set, m_backRightMotor::set
  );

  private final Translation2d m_frontLeftLocation = new Translation2d();
  private final Translation2d m_frontRightLocation = new Translation2d();
  private final Translation2d m_backLeftLocation = new Translation2d();
  private final Translation2d m_backRightLocation = new Translation2d();

  /*
  private final PIDController m_frontLeftPIDController = new PIDController(1, 0, 0);
  private final PIDController m_frontRightPIDController = new PIDController(1, 0, 0);
  private final PIDController m_backLeftPIDController = new PIDController(1, 0, 0);
  private final PIDController m_backRightPIDController = new PIDController(1, 0, 0);
  */

  private final ADXRS450_Gyro m_gyro = new ADXRS450_Gyro();

  private final MecanumDriveKinematics m_kinematics =
      new MecanumDriveKinematics(m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  private final MecanumDriveOdometry m_odometry =
      new MecanumDriveOdometry(
          m_kinematics,
          m_gyro.getRotation2d(),
          new MecanumDriveWheelPositions(
            m_frontLeftEncoder.getPosition(),
            m_frontRightEncoder.getPosition(),
            m_backLeftEncoder.getPosition(),
            m_backRightEncoder.getPosition()
          )
      );

  /* Constructs a MecanumDrive and resets the gyro. */
  public Drivetrain() {
    m_frontRightMotor.setInverted(false);
    m_backRightMotor.setInverted(false);
    m_gyro.reset();
    m_gyro.calibrate();
  }

  public double getGyroAngle() {
    return -m_gyro.getAngle();
  }
  public Rotation2d getGyroRotaion2d() {
    return m_gyro.getRotation2d().times(-1);
  }

  /** Makes the drive method
   * 
   * @param getY Forward movement
   * @param getX Lateral movement
   * @param getZ rotation movement
   * @param driveString Changes driving base based on chosen option
   */
  public void drive(double getY, double getX, double getZ, String driveString) {
    getY = getY * DriveTrainConstants.MAX_SPEED;
    getX = getX * DriveTrainConstants.MAX_SPEED;
    getZ = getZ * DriveTrainConstants.ROTATION_MAX_SPEED;

    switch (driveString) {
      case "robotCentric":
        m_robotDrive.driveCartesian(getY, getX, getZ);
        break;
      case "fieldRelative":
        m_robotDrive.driveCartesian(getY, getX, getZ, getGyroRotaion2d());
        break;
      case "polar":
        m_robotDrive.drivePolar(getY, getGyroRotaion2d(), getZ);
        break;
    
      default:
        break;
    }
    SmartDashboard.putNumber("GetY", getY);
    SmartDashboard.putNumber("GetX", getX);
    SmartDashboard.putNumber("GetZ", getZ);
  }

  /** Updates the field relative position of the robot. */
  /*
  public void updateOdometry() {
    m_odometry.update(m_gyro.getRotation2d(), getCurrentDistances());
  }
  */


































  /**
   * Returns the current state of the drivetrain.
   *
   * @return The current state of the drivetrain.
   */
  /*
  public MecanumDriveWheelSpeeds getCurrentState() {
    return new MecanumDriveWheelSpeeds(
        m_frontLeftEncoder.getVelocity(),
        m_frontRightEncoder.getVelocity(),
        m_backLeftEncoder.getVelocity(),
        m_backRightEncoder.getVelocity()
    );
  }
  */

  /**
   * Set the desired speeds for each wheel.
   *
   * @param speeds The desired wheel speeds.
   */
  /*
  public void setSpeeds(MecanumDriveWheelSpeeds speeds) {
    final double frontLeftFeedforward = m_feedforward.calculate(speeds.frontLeftMetersPerSecond);
    final double frontRightFeedforward = m_feedforward.calculate(speeds.frontRightMetersPerSecond);
    final double backLeftFeedforward = m_feedforward.calculate(speeds.rearLeftMetersPerSecond);
    final double backRightFeedforward = m_feedforward.calculate(speeds.rearRightMetersPerSecond);

    final double frontLeftOutput =
        m_frontLeftPIDController.calculate(
            m_frontLeftEncoder.getVelocity(), speeds.frontLeftMetersPerSecond);
    final double frontRightOutput =
        m_frontRightPIDController.calculate(
            m_frontRightEncoder.getVelocity(), speeds.frontRightMetersPerSecond);
    final double backLeftOutput =
        m_backLeftPIDController.calculate(
            m_backLeftEncoder.getVelocity(), speeds.rearLeftMetersPerSecond);
    final double backRightOutput =
        m_backRightPIDController.calculate(
            m_backRightEncoder.getVelocity(), speeds.rearRightMetersPerSecond);

    m_frontLeftMotor.setVoltage(frontLeftOutput + frontLeftFeedforward);
    m_frontRightMotor.setVoltage(frontRightOutput + frontRightFeedforward);
    m_backLeftMotor.setVoltage(backLeftOutput + backLeftFeedforward);
    m_backRightMotor.setVoltage(backRightOutput + backRightFeedforward);
  }
  */

  /**
   * Old method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  /*
  public void drive(
      double xSpeed, double ySpeed, double rot, boolean fieldRelative, double periodSeconds) {
    var mecanumDriveWheelSpeeds =
        m_kinematics.toWheelSpeeds(
            ChassisSpeeds.discretize(
                fieldRelative
                    ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        xSpeed, ySpeed, rot, m_gyro.getRotation2d())
                    : new ChassisSpeeds(xSpeed, ySpeed, rot),
                periodSeconds));
    mecanumDriveWheelSpeeds.desaturate(kMaxSpeed);
    setSpeeds(mecanumDriveWheelSpeeds);
  }
  */
}
