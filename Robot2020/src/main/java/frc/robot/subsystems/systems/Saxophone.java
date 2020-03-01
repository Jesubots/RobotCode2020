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
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class Saxophone extends SubsystemBase {

  private WPI_TalonSRX flywheel_master = new WPI_TalonSRX(5);
  private WPI_TalonSRX flywheel_follower = new WPI_TalonSRX(4);

  /**
   * Creates a new Saxophone.
   */
  
  public Saxophone() {
    flywheel_follower.set(ControlMode.Follower, 5);
    flywheel_master.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
    flywheel_master.setSensorPhase(true);
    flywheel_master.configNominalOutputForward(0, 30);
    flywheel_master.configNominalOutputReverse(0, 30);
    flywheel_master.configPeakOutputForward(1, 30);
    flywheel_master.configPeakOutputReverse(-1, 30);

    flywheel_master.config_kF(0, Constants.flywheel_kF, 30);
    flywheel_master.config_kP(0, Constants.flywheel_kP, 30);
    flywheel_master.config_kI(0, Constants.flywheel_kI, 30);
    flywheel_master.config_kD(0, Constants.flywheel_kD, 30);

    //flywheel_follower.setInverted(true);
  }

  @Override
  public void periodic() {
    if(RobotContainer.driver_stick.getRawButton(1)) {
      driveFlywheelTargetVelocity();
    } else if(RobotContainer.driver_stick.getRawButton(2)){
      driveFlywheelRawOutput(RobotContainer.driver_stick.getThrottle());
    } else { 
      driveFlywheelRawOutput(0);
    }
    
  }

  public void driveFlywheelTargetVelocity() {
    flywheel_master.set(ControlMode.Velocity, (1000 / 600)*4096);
    flywheel_follower.set(ControlMode.Follower, 5);
  }

  public void driveFlywheelRawOutput(double output) {
    flywheel_master.set(ControlMode.PercentOutput, output);
    flywheel_follower.set(ControlMode.Follower, 5);
  }

  public double getFlywheelVelocity() {
    System.out.println(flywheel_master.getSelectedSensorVelocity());
    return ((double)(flywheel_master.getSelectedSensorVelocity()) / 4096) * 600;
  }
}
