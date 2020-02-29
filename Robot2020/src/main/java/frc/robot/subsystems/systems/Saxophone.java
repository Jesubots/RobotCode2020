/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.systems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class Saxophone extends SubsystemBase {

  private WPI_TalonSRX flywheel = new WPI_TalonSRX(5);

  /**
   * Creates a new Saxophone.
   */
  
  public Saxophone() {
    flywheel.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
  }

  @Override
  public void periodic() {
    driveFlywheelRawOutput(Robot.m_robotContainer.driver_stick.getThrottle());
  }

  public void driveFlywheelRawOutput(double output) {
    flywheel.set(ControlMode.PercentOutput, output);
  }

  public double getFlywheelVelocity() {
    return flywheel.getSelectedSensorVelocity();
  }
}
