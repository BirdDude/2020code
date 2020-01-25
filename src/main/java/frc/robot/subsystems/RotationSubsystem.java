/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class RotationSubsystem extends PIDSubsystem {
  /**
   * Creates a new ExampleSubsystem.
   */

  private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(Constants.sVolts, Constants.vVolts);

  private DriveSubsystem m_driveSubsystem;


  public RotationSubsystem(DriveSubsystem driveSubsystem) {
    super(new PIDController(Constants.tP, Constants.tI, Constants.tD));
    m_driveSubsystem = driveSubsystem;

    getController().setTolerance(Constants.maxRotDeg);
    getController().enableContinuousInput(-180.0, 180.0);
    disable();
  }

  public void setSetpoint(Double angle) {
    setSetpoint(angle);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void useOutput(double output, double setpoint) {
    Double speed = output + m_turnFeedforward.calculate(setpoint);
    m_driveSubsystem.driveCartesan(0.0, 0.0, speed);
  }

  @Override
  public double getMeasurement() {
    return m_driveSubsystem.getGyro().getAngle();
  }
}
