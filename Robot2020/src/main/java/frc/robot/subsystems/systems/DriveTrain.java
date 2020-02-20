/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.systems;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.music.Orchestra;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commands.driveTrain.ArcadeDrive;

public class DriveTrain extends SubsystemBase {
  
  private static double enc_res = 4096;
  private static double wheel_rad = 0.0762; //meters

  public WPI_TalonSRX left_front = new WPI_TalonSRX(1);
  public WPI_TalonSRX left_follower = new WPI_TalonSRX(3);
  public WPI_TalonSRX right_front = new WPI_TalonSRX(2);
  public WPI_TalonSRX right_follower = new WPI_TalonSRX(0);
  private SpeedControllerGroup dt_left = new SpeedControllerGroup(left_follower, left_front);
  private SpeedControllerGroup dt_right = new SpeedControllerGroup(right_follower, right_front);

  private DifferentialDrive driveTrain = new DifferentialDrive(dt_left, dt_right);
  private final DifferentialDriveOdometry dt_odometry;
  private final DifferentialDriveKinematics dt_kinematics;
  private ChassisSpeeds speeds = new ChassisSpeeds();
  private final AHRS ahrs = new AHRS();
  private Joystick driver_stick = Robot.m_robotContainer.driver_stick;

  private boolean speedUpdater = false;
  private double offLeftVal = 0;
  private double offRightVal = 0;
  private double onLeftVal = 0;
  private double onRightVal = 0;
  private double leftSpeed = 0;
  private double rightSpeed = 0;

  public DriveTrain() {
    left_follower.set(ControlMode.Follower, 0);
    right_follower.set(ControlMode.Follower, 2);

    dt_odometry = new DifferentialDriveOdometry(getHeading());
    dt_kinematics = new DifferentialDriveKinematics(Constants.kTrackWidthMeters); 

    //driveTrain.setSafetyEnabled(true);

    setDefaultCommand(new ArcadeDrive());
  }

  @Override
  public void periodic() {
    updateOdometry();
    if(speedUpdater)
      updateSpeedOn();
    else
      updateSpeedOff();
  }

  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(ahrs.getAngle());
  }

  public double getLeftEncoder() {
    left_front.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    return left_front.getSelectedSensorPosition() / enc_res * 2 * wheel_rad * Math.PI / Constants.kDriveTrainGearRatio;
  }

  public double getRightEncoder() {
    right_front.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    return right_front.getSelectedSensorPosition() / enc_res * 2 * wheel_rad * Math.PI / Constants.kDriveTrainGearRatio;
  }

  public double getLeftSpeed() {
    return leftSpeed;
  }

  public double getRightSpeed() {
    return rightSpeed;
  }

  public Pose2d getPose() {
    return dt_odometry.getPoseMeters();
  }

  public void zeroHeading() {
    ahrs.reset();
  }

  public void arcadeDrive(double leftInput, double twistInput) {
    driveTrain.arcadeDrive(leftInput, twistInput);
  }

  public void updateOdometry() {
    dt_odometry.update(getHeading(), getLeftEncoder(), getRightEncoder());
  }

  private void updateSpeedOn() {
    leftSpeed = (getLeftEncoder() - offLeftVal) / .02;
    rightSpeed = (getRightEncoder() - offRightVal) / .02;
    onLeftVal = getLeftEncoder();
    onRightVal = getRightEncoder();
  }

  private void updateSpeedOff() {
    leftSpeed = (getLeftEncoder() - onLeftVal) / .02;
    rightSpeed = (getRightEncoder() - onRightVal) / .02;
    offLeftVal = getLeftEncoder();
    offRightVal = getRightEncoder();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeftSpeed(), getRightSpeed());
  }

  public void tankDriveVolts(double leftVolts, double rightVolts){
    left_front.setVoltage(leftVolts);
    left_front.setVoltage(-rightVolts);
    driveTrain.feed();
  }
  

}
