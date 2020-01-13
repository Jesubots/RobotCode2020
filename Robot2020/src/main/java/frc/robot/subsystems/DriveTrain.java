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


  private WPI_TalonSRX left_front_talon = new WPI_TalonSRX(0);
  private WPI_TalonSRX left_back_talon = new WPI_TalonSRX(1);
  private WPI_TalonSRX right_front_talon = new WPI_TalonSRX(2);
  private WPI_TalonSRX right_back_talon = new WPI_TalonSRX(3);

  private SpeedControllerGroup dt_left = new SpeedControllerGroup(left_back_talon, left_front_talon);
  private SpeedControllerGroup dt_right = new SpeedControllerGroup(right_back_talon, right_front_talon);

  private DifferentialDrive driveTrain = new DifferentialDrive(dt_left, dt_right);

  public DriveTrain() {

  }

  @Override
  public void periodic() {
    driveTrain.tankDrive(Robot.driver_stick.getY(), Robot.driver_stick_2.getY());
  }
}
