/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.JoystickSubsystem;

/**
 * An example command that uses an example subsystem.
 */
public class DefaultDrive extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveSubsystem m_DriveSubsystem;
  private final JoystickSubsystem m_joystickSubsystem;

  /**
   * Creates a new ExampleCommand.
   *
   * subsystem The subsystem used by this command.
   */
  public DefaultDrive(DriveSubsystem driveSubsystem, JoystickSubsystem joystickSubsystem) {
    m_DriveSubsystem = driveSubsystem;
    m_joystickSubsystem = joystickSubsystem;
    // Use addRequirements() here to declare subsyste m dependencies.
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_DriveSubsystem.driveCartesan(m_joystickSubsystem.getJoystick().getX(), m_joystickSubsystem.getJoystick().getY(), m_joystickSubsystem.getJoystick().getTwist());

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
