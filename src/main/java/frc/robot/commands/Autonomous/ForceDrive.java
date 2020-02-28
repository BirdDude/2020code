/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.ExampleClasses.ExampleSubsystem;
import frc.robot.subsystems.Drivetrain.DriveSubsystem;

/**
 * An example command that uses an example subsystem.
 */
public class ForceDrive extends InstantCommand {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveSubsystem m_driveSubsystem;
  private Double x, y, r;


  public ForceDrive(DriveSubsystem subsystem, Double x, Double y, Double r) {
    m_driveSubsystem = subsystem;
    this.x = x;
    this.y = y;
    this.r = r;

    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    m_driveSubsystem.driveCartesan(x, y, r);
  }

}
