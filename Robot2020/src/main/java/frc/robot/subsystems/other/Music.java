/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.other;

import java.util.ArrayList;
import java.util.Collection;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.music.Orchestra;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Music extends SubsystemBase {

  private Collection<TalonFX> falcons = new ArrayList<TalonFX>();
  public Orchestra midi;

  /**
   * Creates a new Orchestra.
   */
  public Music() {
  }

  @Override
  public void periodic() {
    
  }
}
