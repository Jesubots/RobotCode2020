/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
    //TALON PORTS
    public static final int CONVEYER_PORT = 0;
    public static final int LOADER_PORT = 2;
    public static final int INTAKE_PORT = 5;
    public static final int FW_MASTER_PORT = 7;
    public static final int FW_FOLLOWER_PORT = 4;
    public static final int DTFL_PORT = 1;
    public static final int DTBL_PORT = 3;
    public static final int DTFR_PORT = 6;
    public static final int DTBR_PORT = 8;


    //DRIVETRAIN CONSTANTS
    public static final double kDriveTrainGearRatio = 4.74;
    public static final double TURN_MULTIPLIER = .5;

    //CHARACTERIZATION CONSTANTS
    public static final double ksVolts = 0.334; 
    public static final double kvVoltSecondsPerMeter = 0.364;
    public static final double kaVoltSecondsSquaredPerMeter = 0.0857;
    public static final double kPDriveVel = 0.00874;
    public static final double kTrackWidthMeters = 0.7112;
    public static final double kMaxSpeedMetersPerSecond = 8;
    public static final double kMaxAccelerationMetersPerSecondSquared = 16;
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackWidthMeters);

    //RAMSETE CONSTANTS (these are not tuned but are, according to WPI, reasonable)
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;

    public static final double flywheel_kF = (.75 * 1023) / 11000;
    public static final double flywheel_kP = .15;
    public static final double flywheel_kI = 0;
    public static final double flywheel_kD = 5;
}
