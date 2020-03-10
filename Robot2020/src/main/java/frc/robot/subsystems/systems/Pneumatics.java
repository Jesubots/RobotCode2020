/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.systems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class Pneumatics extends SubsystemBase {

  public Compressor compressor = new Compressor();
  private Solenoid shift_up_sol = new Solenoid(3);
  private Solenoid shift_down_sol = new Solenoid(4);
  private Solenoid shoot_frwd_sol = new Solenoid(2);
  private Solenoid shoot_rvrs_sol = new Solenoid(5);
  private Solenoid spin_frwd_sol = new Solenoid(1);
  private Solenoid spin_rvrs_sol = new Solenoid(6);
  private Solenoid lift_frwd_sol = new Solenoid(0);
  private Solenoid lift_rvrs_sol = new Solenoid(7);

  private boolean liftIsUp = false;
  private boolean shiftedUp = false;
  private boolean spinIsUp = false;
  private boolean shooterIsOut = false;
  /**
   * Creates a new Pneumatics.
   */
  public Pneumatics() {

  }

  @Override
  public void periodic() {
    if(RobotContainer.driver_stick.getRawButtonPressed(5)) {
      if(liftIsUp)
        liftDown();
      else 
        liftUp();
    }
    if(RobotContainer.driver_stick.getRawButtonPressed(6)) {
      if(shiftedUp)
        shiftDown();
      else 
        shiftUp();
    }
    if(RobotContainer.driver_stick.getRawButtonPressed(4)) {
      if(spinIsUp)
        spinnerDown();
      else 
        spinnerUp();
    }
    if(RobotContainer.driver_stick.getRawButtonPressed(7)) {
      if(shooterIsOut)
        shooterIn();
      else 
        shooterOut();
    }
  }

  public void shiftUp() {
    shift_down_sol.set(false);
    shift_up_sol.set(true);
    shiftedUp = true;
  }

  public void shiftDown() {
    shift_up_sol.set(false);
    shift_down_sol.set(true);
    shiftedUp = false;
  }

  public void liftDown() {
    lift_frwd_sol.set(false);
    lift_rvrs_sol.set(true);
    liftIsUp = false;
  }

  public void liftUp() {
    lift_rvrs_sol.set(false);
    lift_frwd_sol.set(true);
    liftIsUp = true;
  }

  public void spinnerUp() {
    spin_rvrs_sol.set(false);
    spin_frwd_sol.set(true);
    spinIsUp = true;
  }

  public void spinnerDown() {
    spin_frwd_sol.set(false);
    spin_rvrs_sol.set(true);
    spinIsUp = false;
  }

  public void shooterOut() {
    shoot_frwd_sol.set(false);
    shoot_rvrs_sol.set(true);
    shooterIsOut = true;
  }

  public void shooterIn() {
    shoot_rvrs_sol.set(false);
    shoot_frwd_sol.set(true);
    shooterIsOut = false;
  }
}
