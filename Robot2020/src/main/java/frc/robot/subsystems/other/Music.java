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
import frc.robot.Robot;

public class Music extends SubsystemBase {
  /*
  private TalonFX left_front = new TalonFX(1);
  private TalonFX left_follower = new TalonFX(0);
  private TalonFX right_front = new TalonFX(2);
  private TalonFX right_follower = new TalonFX(1);

  private Collection<TalonFX> falcons = new ArrayList<TalonFX>();
  public Orchestra midi;

  /**
   * Creates a new Orchestra.
   
  public Music() {
    midi = new Orchestra(falcons);
    midi.addInstrument(right_front);
    midi.addInstrument(right_follower);
    midi.addInstrument(left_front);
    midi.addInstrument(left_follower);
    midi.loadMusic("notsus.chrp");
  }

  @Override
  public void periodic() {
    if(Robot.driver_stick.getRawButton(1)) {
      System.out.println("start song");
      midi.play();
    }
    if(Robot.driver_stick.getRawButton(2)) {
      midi.pause();
    }
    if(Robot.driver_stick.getRawButton(3)) {
      midi.stop();
    }
    
  }
  */
}
