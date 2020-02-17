/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.PowerCells;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.ExampleClasses.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * An example command that uses an example subsystem.
 */
public class RotateIntakeBarTo extends PIDCommand {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final IntakeSubsystem m_intakeSubsystem;


  public RotateIntakeBarTo(Double target, IntakeSubsystem intakeSubsystem) {
    super (
            new PIDController(0.3, 0.0, 0.0),

            // Close loop on heading
            intakeSubsystem::getDeployTicks,

            // Set reference to target
            target,
            // Pipe output to turn robot
            output -> intakeSubsystem.m_intakeDeploy.set(output),
            // Require the drive
            intakeSubsystem
    );

    getController().setTolerance(30, 20);

    m_intakeSubsystem = intakeSubsystem;
    addRequirements(intakeSubsystem);
  }

  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
