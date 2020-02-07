/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class IntakeSubsystem extends SubsystemBase {

  WPI_TalonSRX m_intake = new WPI_TalonSRX(Constants.intakePort);
  WPI_TalonSRX m_intakeDeploy = new WPI_TalonSRX(Constants.intakeDeployPort);

  public IntakeSubsystem() {

  }

  @Override
  public void periodic() { }


  void runIntake() {
    m_intake.set(Constants.intakePower);
  }


}
