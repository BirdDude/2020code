/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.PowerCells;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TransportSubsystem extends SubsystemBase {

  public WPI_TalonSRX m_storage = new WPI_TalonSRX(Constants.transportPort);

  public TransportSubsystem() {
    m_storage.setInverted(true);
  }

  public void setPower(double power) {
    m_storage.set(power);
  }

  @Override
  public void periodic() { }

}
