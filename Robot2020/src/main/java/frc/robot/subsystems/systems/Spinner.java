/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.systems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Spinner extends SubsystemBase {

  private WPI_TalonSRX wheel_fondler = new WPI_TalonSRX(9);
  private final I2C.Port i2cport = I2C.Port.kOnboard; 
  //private final ColorSensorV3 colorSensor = new ColorSensorV3(i2cport);

  /**
   * Creates a new Spinner.
   */
  public Spinner() {
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Color readColor() {
    return new Color(0, 0, 0);//colorSensor.getColor();
  }
}
