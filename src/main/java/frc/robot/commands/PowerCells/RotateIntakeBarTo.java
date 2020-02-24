/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.PowerCells;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.PowerCells.IntakeSubsystem;

/**
 * An example command that uses an example subsystem.
 */
public class RotateIntakeBarTo extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final IntakeSubsystem m_intakeSubsystem;

  private Double target = Constants.upPosEncoderTicks;
  private PIDController m_controller = new PIDController(0.45, 0.0, 0.0);


  public RotateIntakeBarTo(IntakeSubsystem intakeSubsystem) {
    m_controller.setSetpoint(target);
//    m_controller.setTolerance(10.0, 5.0);

    m_intakeSubsystem = intakeSubsystem;
  }

  @Override
  public void execute() {
    m_controller.setSetpoint(target);
    Double power = m_controller.calculate(m_intakeSubsystem.intakeEncoderTicks, target) / Constants.downPosEncoderTicks;

    if (Math.abs(power) < 0.1) power = 0.0;

    System.out.println(power);
    m_intakeSubsystem.m_intakeDeploy.set(power);
  }

  @Override
  public boolean isFinished() { return false; }

  public class moveUp extends InstantCommand {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

    @Override
    public void initialize() {
      target = Constants.upPosEncoderTicks;
    }
  }
  public class moveDown extends InstantCommand {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

    @Override
    public void initialize() {
      target = Constants.downPosEncoderTicks;    }
  }

}



