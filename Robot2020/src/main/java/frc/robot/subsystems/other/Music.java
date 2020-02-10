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
  
  private TalonFX left_front = new TalonFX(0);
  private TalonFX left_follower = new TalonFX(1);
  private TalonFX right_front = new TalonFX(2);
  private TalonFX right_follower = new TalonFX(3);

  private Collection<TalonFX> falcons = new ArrayList<TalonFX>();
  public Orchestra midi;

  /**
   * Creates a new Orchestra.
   */
  public Music() {
    falcons.add(left_front);
    falcons.add(right_front);
    falcons.add(left_follower);
    falcons.add(right_follower);
    midi = new Orchestra(falcons);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
