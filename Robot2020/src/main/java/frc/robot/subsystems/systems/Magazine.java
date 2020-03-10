/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.systems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Magazine extends SubsystemBase {

  private WPI_TalonSRX loader = new WPI_TalonSRX(Constants.LOADER_PORT);
  private WPI_TalonSRX conveyer = new WPI_TalonSRX(Constants.CONVEYER_PORT);

  /**
   * Creates a new Magazine.
   */
  public Magazine() {
    conveyer.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void periodic() {

  }

  public void runConveyer(double output) {
    conveyer.set(ControlMode.PercentOutput, output);
  }

  public void stopConveyer() {
    conveyer.stopMotor();
  }

  public void runLoader(double output) {
    loader.set(ControlMode.PercentOutput, output);
  }

  public void stopLoader() {
    loader.stopMotor();
  }
}
