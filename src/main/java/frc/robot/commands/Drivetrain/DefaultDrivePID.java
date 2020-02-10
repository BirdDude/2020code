/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Drivetrain;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.ExampleClasses.ExampleSubsystem;

/**
 * An example command that uses an example subsystem.
 */
public class DefaultDrivePID extends CommandBase {

  private PIDController fL = new PIDController(0.0, 0.0, 0.0);
  private PIDController fR = new PIDController(0.0, 0.0, 0.0);
  private PIDController bL = new PIDController(0.0, 0.0, 0.0);
  private PIDController bR = new PIDController(0.0, 0.0, 0.0);
  private Joystick joystick;

  DefaultDrivePID() {

  }

  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})


  public DefaultDrivePID(ExampleSubsystem subsystem) {

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
