/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

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
    }

    public static final class ArmConstants {
        //3937,3944 - armPostion in top position
        //4018 - straight up
        //4306 - 75deg
        //4619 - 40 deg
        //5098-9,5109 - armPosition in bottom position
        public static final int kArmP = 1;
        public static final int kArmI = 2;
        public static final int kArmD = 0;
    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
    }
}
