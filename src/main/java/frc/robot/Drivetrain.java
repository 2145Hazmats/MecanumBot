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
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Represents a mecanum drive style drivetrain. */
public class Drivetrain {
  //makes new CAN sparkmax objects
  private final CANSparkMax m_frontLeftMotor = new CANSparkMax(1, MotorType.kBrushless);
  private final CANSparkMax m_frontRightMotor = new CANSparkMax(2, MotorType.kBrushless);
  private final CANSparkMax m_backLeftMotor = new CANSparkMax(3, MotorType.kBrushless);
  private final CANSparkMax m_backRightMotor = new CANSparkMax(4, MotorType.kBrushless);
  //makes new Encoder objects
  private final RelativeEncoder m_frontLeftEncoder = m_frontLeftMotor.getEncoder();
  private final RelativeEncoder m_frontRightEncoder = m_frontRightMotor.getEncoder();
  private final RelativeEncoder m_backLeftEncoder = m_backLeftMotor.getEncoder();
  private final RelativeEncoder m_backRightEncoder = m_backRightMotor.getEncoder();

  //makes our complicated gyro readable by the robot
  private final ADXRS450_Gyro m_gyro = new ADXRS450_Gyro();

  //makes an object of the mecanumdrive class, we use drive methods of this object
  private MecanumDrive m_robotDrive = new MecanumDrive(
      // The parameters are consumers to set the speed of the motors
      m_frontLeftMotor::set, m_backLeftMotor::set, m_frontRightMotor::set, m_backRightMotor::set
  );

  //tells the robot where the wheel positions are
  private final Translation2d m_frontLeftLocation = new Translation2d(0.254, 0.2667);
  private final Translation2d m_frontRightLocation = new Translation2d(0.254, -0.2667);
  private final Translation2d m_backLeftLocation = new Translation2d(-0.254, 0.2667);
  private final Translation2d m_backRightLocation = new Translation2d(-0.254, -0.2667);
  // makes a kinematics object of the mecanum drive using wheel positions
  private final MecanumDriveKinematics m_kinematics =
      new MecanumDriveKinematics(m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);
  // makes a MecanumDriveOdometry object that tells the robot where it is using motor encoders, gyro, and math
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

  // places the field into smartDashboard. We use this to place the robot's position on smartdashboard
  private final Field2d m_field = new Field2d();

  /* Constructs a MecanumDrive and resets the gyro. */
  public Drivetrain() {
    m_frontLeftMotor.restoreFactoryDefaults();
    m_frontLeftMotor.restoreFactoryDefaults();
    m_backLeftMotor.restoreFactoryDefaults();
    m_backRightMotor.restoreFactoryDefaults();

    m_frontLeftEncoder.setPositionConversionFactor(DriveTrainConstants.ENCODER_CONVERSION_FACTOR);
    m_frontRightEncoder.setPositionConversionFactor(DriveTrainConstants.ENCODER_CONVERSION_FACTOR);
    m_backLeftEncoder.setPositionConversionFactor(DriveTrainConstants.ENCODER_CONVERSION_FACTOR);
    m_backRightEncoder.setPositionConversionFactor(DriveTrainConstants.ENCODER_CONVERSION_FACTOR);

    m_gyro.reset();     // sets the gyro angle to 0 degrees
    m_gyro.calibrate(); // calibrates the gyro
    SmartDashboard.putData("Field", m_field);
  }

  /* returns the inverted gyro angle for proper rotation */
  public double getGyroAngle() {
    return -m_gyro.getAngle();
  }

  /* returns the inverted gyro rotation2d */
  public Rotation2d getGyroRotaion2d() {
    return m_gyro.getRotation2d().times(-1);
  }

  /** Mecanum drive method
   * 
   * @param getY Forward movement
   * @param getX Lateral movement
   * @param getZ rotation movement
   * @param driveString Changes driving base based on chosen option
   */
  public void drive(double getY, double getX, double getZ, String driveString) {
    // Limits the max speed and rotation speeds
    getY = getY * DriveTrainConstants.MAX_SPEED;
    getX = getX * DriveTrainConstants.MAX_SPEED;
    getZ = getZ * DriveTrainConstants.ROTATION_MAX_SPEED;

    // allows us to choose different drive methods. we select/change driveString in smartdashboard to choose different drive types
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
      default: // should not run, but included to avoid errors during an invalid driveString
        break;
    }

    // places forward and sideways speed, as well as rotation of the robot into SmartDashboard
    SmartDashboard.putNumber("GetY", getY);
    SmartDashboard.putNumber("GetX", getX);
    SmartDashboard.putNumber("GetZ", getZ);
  }

  /* Called periodically in Robot.java */
  public void drivetrainPeriodic() {
    // Gets the encoder position for each wheel
    MecanumDriveWheelPositions wheelPositions = new MecanumDriveWheelPositions(
      m_frontLeftEncoder.getPosition(),
      m_frontRightEncoder.getPosition(), 
      m_backLeftEncoder.getPosition(),
      m_backRightEncoder.getPosition()
    );
    // updates the odometry using the current wheel positions compared to old wheel positions and GyroRotation2d
    m_odometry.update(m_gyro.getRotation2d(), wheelPositions);

    // places the wheel positions and gyro angle into SmartDashboard
    SmartDashboard.putNumber("Encoder position FrontLeft", m_frontLeftEncoder.getPosition());
    SmartDashboard.putNumber("Encoder position FrontRight", m_frontRightEncoder.getPosition());
    SmartDashboard.putNumber("Encoder position BackLeft", m_backLeftEncoder.getPosition());
    SmartDashboard.putNumber("Encoder position BackRight", m_backRightEncoder.getPosition());
    SmartDashboard.putNumber("Gyro Angle", getGyroAngle());
    // publishes the robot pose and updates it onto the field in SmartDashboard
    m_field.setRobotPose(m_odometry.getPoseMeters());
  }

}
