/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.systems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
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
import frc.robot.commands.drivetrain.ArcadeDrive;

public class DriveTrain extends SubsystemBase {
  
  private static double enc_res = 4096;
  private static double wheel_rad = 0.0762; //meters

  public WPI_TalonFX left_front = new WPI_TalonFX(Constants.DTFL_PORT);
  public WPI_TalonFX left_follower = new WPI_TalonFX(Constants.DTBL_PORT);
  public WPI_TalonFX right_front = new WPI_TalonFX(Constants.DTFR_PORT);
  public WPI_TalonFX right_follower = new WPI_TalonFX(Constants.DTBR_PORT);
  private SpeedControllerGroup dt_left = new SpeedControllerGroup(left_follower, left_front);
  private SpeedControllerGroup dt_right = new SpeedControllerGroup(right_follower, right_front);

  private DifferentialDrive driveTrain = new DifferentialDrive(dt_left, dt_right);
  private final DifferentialDriveOdometry dt_odometry;
  private final DifferentialDriveKinematics dt_kinematics;
  private ChassisSpeeds speeds = new ChassisSpeeds();
  private final AHRS ahrs = new AHRS();
  private Joystick driver_stick = RobotContainer.driver_stick;

  public DriveTrain() {
    left_front.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    right_front.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    
    left_front.setNeutralMode(NeutralMode.Brake);
    right_front.setNeutralMode(NeutralMode.Brake);
    left_follower.setNeutralMode(NeutralMode.Brake);
    right_follower.setNeutralMode(NeutralMode.Brake);


    dt_odometry = new DifferentialDriveOdometry(getHeading());
    dt_kinematics = new DifferentialDriveKinematics(Constants.kTrackWidthMeters); 

    //driveTrain.setSafetyEnabled(true);

  }

  @Override
  public void periodic() {
    arcadeDrive(-driver_stick.getY(), driver_stick.getTwist() * Constants.TURN_MULTIPLIER);
  }

  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(ahrs.getAngle());
  }

  public double getLeftEncoder() {
    return (left_front.getSelectedSensorPosition() / enc_res) * (2 * wheel_rad * Math.PI) / Constants.kDriveTrainGearRatio;
  }

  public double getRightEncoder() {
    return (right_front.getSelectedSensorPosition() / enc_res) * (2 * wheel_rad * Math.PI) / Constants.kDriveTrainGearRatio;
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

  public double getRobotVelocity() {
    return ((((double)left_front.getSelectedSensorVelocity() - (double)right_front.getSelectedSensorVelocity())/ 4096f) * .0508 * 2 * Math.PI * 10f);
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(left_front.getSelectedSensorVelocity(), right_front.getSelectedSensorVelocity());
  }

  public void tankDriveVolts(double leftVolts, double rightVolts){
    left_front.setVoltage(leftVolts);
    left_front.setVoltage(rightVolts);
    left_follower.setVoltage(leftVolts);
    right_follower.set(rightVolts);
    driveTrain.feed();
  }
  

}
