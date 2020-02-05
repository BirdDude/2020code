/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class ClimberSubsystem extends SubsystemBase {
 
  TanlonSRX rotor = new TalonSRX(Constants.climbRotor);
  TanlonSRX winch = new TalonSRX(Constants.climbWinch);

  public ClimberSubsystem() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  void climb() {
    winch.set(ControlMode.PercentOutput, Constants.climbWinchPower);
  }

  void extend() {
    rotor.set(ControlMode.PercentOutput, Constants.climbRotorPower);
  }

  void retract() {
    rotor.set(ControlMode.PercentOutput, -Constants.climbRotorPower);
  }
}
