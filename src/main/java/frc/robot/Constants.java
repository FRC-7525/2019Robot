/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class DriveConstants {
        public static final int kLeftMotor1 = 1;
        public static final int kRightMotor1 = 0;
        public static final int kLeftMotor2 = 3;
        public static final int kRightMotor2 = 2;

        public static final int kShifterSolenoid = 0;

        public static final double WHEEL_DIAMETER = 0.1016; // 4in in meters
        public static final double ENCODER_EDGES_PER_REV = 4096;
        public static final int PIDIDX = 0;
        public static final double encoderConstant = (1 / ENCODER_EDGES_PER_REV) * WHEEL_DIAMETER * Math.PI;

        public static final double ksVolts = 0.951;
        public static final double kvVoltSecondsPerMeter = 4.21;
        public static final double kaVoltSecondsSquaredPerMeter = 0.837;

        public static final double kPDriveVel = 10; //20.4; Needs to be tuned

        public static final double kTrackwidthMeters = 0.6748;
        public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 1.0;
        public static final double kMaxAccelerationMetersPerSecondSquared = 0.5;

        // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;
    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
        //private final Joystick m_stick = new Joystick(0); //TODO: move wherever Command Based has it.
      }
}
