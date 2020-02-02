/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class DriveTrain extends SubsystemBase {


  private WPI_TalonSRX left_front = new WPI_TalonSRX(0);
  private WPI_TalonSRX left_follower = new WPI_TalonSRX(1);
  private WPI_TalonSRX right_front = new WPI_TalonSRX(2);
  private WPI_TalonSRX right_follower = new WPI_TalonSRX(3);`

  private SpeedControllerGroup dt_left = new SpeedControllerGroup(left_follower, left_front);
  private SpeedControllerGroup dt_right = new SpeedControllerGroup(right_follower, right_front);

  private DifferentialDrive driveTrain = new DifferentialDrive(dt_left, dt_right);

  public DriveTrain() {
    left_follower.setControlMode(ControlMode.follower, 0);
    right_follower.setControlMode(ControlMode.follower, 2);
    left_front.
  }

  @Override
  public void periodic() {
    driveTrain.tankDrive(Robot.driver_stick.getY(), Robot.driver_stick_2.getY());
  }

  public double getDistance() {

  }
}
