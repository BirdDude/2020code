/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class IntakeSubsystem extends SubsystemBase {

  public WPI_TalonSRX m_intakeDeploy = new WPI_TalonSRX(Constants.intakeDeployPort);
  public WPI_VictorSPX m_intake = new WPI_VictorSPX(Constants.intakePort);

  public IntakeSubsystem() {

  }

  @Override
  public void periodic() { }


}
