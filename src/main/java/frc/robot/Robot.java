// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.Constants.OperatorConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
  private final XboxController m_controller = new XboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);
  private final Drivetrain m_mecanum = new Drivetrain(); // makes a new Drivetrain object

  private SendableChooser<String> m_driveSendableChooser = new SendableChooser<String>(); // Allows choosing of the drive method
  
  //Limits the rate of change of speed of the robot. Each axis needs its own object of SlewRateLimiter
  private final SlewRateLimiter m_xAccelerationLimiter = new SlewRateLimiter(DriveTrainConstants.ACCELERATION_LIMIT_RATE);
  private final SlewRateLimiter m_xDeAccelerationLimiter = new SlewRateLimiter(DriveTrainConstants.DEACCELERATION_LIMIT_RATE);
  private final SlewRateLimiter m_yAccelerationLimiter = new SlewRateLimiter(DriveTrainConstants.ACCELERATION_LIMIT_RATE);
  private final SlewRateLimiter m_yDeAccelerationLimiter = new SlewRateLimiter(DriveTrainConstants.DEACCELERATION_LIMIT_RATE);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriveTrainConstants.ROTATION_LIMIT_RATE);
 
  /* As the robot is turned on, places these values into ShuffleBoard. This method is called once */
  @Override
  public void robotInit() {
    SmartDashboard.putBoolean("fieldRelative", true);
    m_driveSendableChooser.setDefaultOption("robotCentric",  "robotCentric");
    m_driveSendableChooser.addOption("fieldRelative", "fieldRelative");
    m_driveSendableChooser.addOption("polar", "polar");
    SmartDashboard.putData("Drive Type", m_driveSendableChooser);
  }

  /* happens every 20 milliseconds regardless of auton or teleop */ 
  @Override
  public void robotPeriodic() {
    m_mecanum.drivetrainPeriodic();
  }

  /* happens every 20 milliseconds while in auton */
  @Override
  public void autonomousPeriodic() { }

  /* happens every 20 milliseconds while in teleop */
  @Override
  public void teleopPeriodic() {
    driveWithJoystick(); // calls the drive method
  }

  /* Drives robot with specific max acceleration and max deacceleration speeds */
  public void driveWithJoystick() {
    m_mecanum.drive(
        //makes a condition (acts like a boolean) that depends on if Left Joystick is at 0.0 on y axis
        MathUtil.applyDeadband(m_controller.getLeftY(), OperatorConstants.LEFT_DEADBAND) == 0
        //If true, Causes the robot to deaccelerate  noninstantly while left stick is in neutral position.
        ? m_yDeAccelerationLimiter.calculate(MathUtil.applyDeadband(m_controller.getLeftY(), OperatorConstants.LEFT_DEADBAND))
        //If false, limits robot's acceleration speed
        : m_yAccelerationLimiter.calculate(MathUtil.applyDeadband(m_controller.getLeftY(), OperatorConstants.LEFT_DEADBAND)),
        //makes a condition (acts like a boolean) that depends on if Left Joystick is at 0.0 on x axis
        MathUtil.applyDeadband(-m_controller.getLeftX(), OperatorConstants.LEFT_DEADBAND) == 0
        //If true, causes the robot to deaccelerate noninstantly while left stick is in neutral position
        ? m_xDeAccelerationLimiter.calculate(MathUtil.applyDeadband(-m_controller.getLeftX(), OperatorConstants.LEFT_DEADBAND))
        //If false, limits robot's acceleration speed
        : m_xAccelerationLimiter.calculate(MathUtil.applyDeadband(-m_controller.getLeftX(), OperatorConstants.LEFT_DEADBAND)),
        //allows robot to rotate with a max acceleration speed
        m_rotLimiter.calculate(MathUtil.applyDeadband(-m_controller.getRightX(), OperatorConstants.RIGHT_DEADBAND)),
        //tells the robot how to drive (drive type) from SmartDashboard
        m_driveSendableChooser.getSelected()
    );
    
  }
  
}