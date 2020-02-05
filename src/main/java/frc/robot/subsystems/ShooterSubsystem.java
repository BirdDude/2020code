/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.Constants
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class ShooterSubsystem extends SubsystemBase {
  
  TalonSRX flyWheel = new TalonSRX(Constants.shooterPort);

  public ShooterSubsystem() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  void startFlywheel() {
    flyWheel.set(ControlMode.PercentOutput, Constants.shooterPower);
  }
  void startFlywheel(double power) {
    flyWheel.set(ControlMode.PercentOutput, power);
  }

  void stopFlyWheel () {
    flyWheel.set(ControlMode.PercentOutput, 0.0);
  }
}
