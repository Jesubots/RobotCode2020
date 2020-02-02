/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class DriveTrain extends SubsystemBase {
  
  private static double enc_res = 4096;
  private static double wheel_rad = 0.0762; //meters

  private WPI_TalonSRX left_front = new WPI_TalonSRX(0);
  private WPI_TalonSRX left_follower = new WPI_TalonSRX(1);
  private WPI_TalonSRX right_front = new WPI_TalonSRX(2);
  private WPI_TalonSRX right_follower = new WPI_TalonSRX(3);
  private SpeedControllerGroup dt_left = new SpeedControllerGroup(left_follower, left_front);
  private SpeedControllerGroup dt_right = new SpeedControllerGroup(right_follower, right_front);

  private DifferentialDrive driveTrain = new DifferentialDrive(dt_left, dt_right);
  private final DifferentialDriveOdometry dt_odometry;
  private final DifferentialDriveKinematics dt_kinematics;
  private ChassisSpeeds speeds = new ChassisSpeeds();
  private final AHRS ahrs = new AHRS();

  public DriveTrain() {
    left_follower.set(ControlMode.Follower, 0);
    right_follower.set(ControlMode.Follower, 2);

    dt_odometry = new DifferentialDriveOdometry(getGyroAngle());
    dt_kinematics = new DifferentialDriveKinematics(Constants.TRACK_WIDTH);
  }

  @Override
  public void periodic() {
    driveTrain.tankDrive(Robot.driver_stick.getY(), Robot.driver_stick_2.getY());
  }

  public Rotation2d getGyroAngle() {
    return Rotation2d.fromDegrees(ahrs.getAngle());
  }

  public double getLeftEncoderDistance() {
    left_front.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    double dist = left_front.getSelectedSensorPosition() / enc_res * 2 * wheel_rad * Math.PI / Constants.DRIVETRAIN_GEAR_RATIO;
    return dist;
  }

  public double getRightEncoderDistance() {
    right_front.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    double dist = right_front.getSelectedSensorPosition() / enc_res * 2 * wheel_rad * Math.PI / Constants.DRIVETRAIN_GEAR_RATIO;
    return dist;
  }

  public Pose2d getPose() {
    return dt_odometry.getPoseMeters();
  }

  public void updateChassisSpeeds() {
    
  }

  public void updateOdometry() {
    dt_odometry.update(getGyroAngle(), getLeftEncoderDistance(), getRightEncoderDistance());
  }
}
