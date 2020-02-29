/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.systems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Spinner extends SubsystemBase {

  private WPI_TalonSRX wheel_fondler = new WPI_TalonSRX(9);

  /**
   * Creates a new Spinner.
   */
  public Spinner() {
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
