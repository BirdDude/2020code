/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Drivetrain.DriveSubsystem;

/**
 * An example command that uses an example subsystem.
 */
public class AutoDrive extends InstantCommand {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveSubsystem m_driveSubsystem;
  private Double x, y, r, t;


  public AutoDrive(DriveSubsystem subsystem, Double x, Double y, Double r, Double time) {
    m_driveSubsystem = subsystem;
    this.x = x;
    this.y = y;
    this.r = r;
    this.t = time;

    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    m_driveSubsystem.autoCartesian(x, y, r);
  }

}
