/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.RotationSubsystem;


/**
 * An example command that uses an example subsystem.
 */
public class RotateTo extends CommandBase {

  private final RotationSubsystem m_rotationSubsystem;
  private final DriveSubsystem m_driveSubsystem;
  Double angle = 0.0;

  /**
   * Creates a new ExampleCommand.
   *
   * param subsystem The subsystem used by this command.
   */

  //@TODO Add PID Values
  public RotateTo(RotationSubsystem rotationSubsystem, DriveSubsystem driveSubsystem) {
    m_rotationSubsystem = rotationSubsystem;
    m_driveSubsystem = driveSubsystem;


  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_rotationSubsystem.setSetpoint(angle);
    m_rotationSubsystem.enable();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_rotationSubsystem.disable();
    if (interrupted) {
      m_driveSubsystem.driveCartesan(0.0, 0.0, 0.0);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_rotationSubsystem.getController().getPositionError() < 3;
  }

  public void setAngle(Double ang) {
    ang = angle;
  }
}
