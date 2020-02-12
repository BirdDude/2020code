/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.MecanumControllerCommand;
import frc.robot.ExampleClasses.ExampleSubsystem;

/**
 * An example command that uses an example subsystem.
 */
public class PathfindTo extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ExampleSubsystem m_subsystem;


  public PathfindTo(ExampleSubsystem subsystem) {
    m_subsystem = subsystem;

    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    System.out.println("Hello World!");
  }

  @Override
  public void execute() {
    System.out.println("Hello World!");
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("Hello World!");
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
