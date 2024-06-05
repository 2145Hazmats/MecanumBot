// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public final class Constants {

    public static class DriveTrainConstants {
        public static final double MAX_SPEED = 0.5; // Max speed (motor % * 100) of robot
        public static final double ROTATION_MAX_SPEED = 0.5; // Max rotation speed (motor % * 100) of robot
        public static final double ACCELERATION_LIMIT_RATE = 5; // Rate-of-change limit for speed, units per second.
        public static final double ROTATION_LIMIT_RATE = 5; // Rate-of-change limit for speed, units per second, rotation-wise
        public static final double DEACCELERATION_LIMIT_RATE = 7; // How fast the robot comes to a stop in seconds: (1/DEACCELERATION)
    }
 
    public static class OperatorConstants {
        public static final int DRIVER_CONTROLLER_PORT  = 0;
        public static final double LEFT_DEADBAND  = 0.03;
        public static final double RIGHT_DEADBAND  = 0.03;
    }

}
