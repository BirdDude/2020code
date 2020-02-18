/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.PowerCells;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * An example command that uses an example subsystem.
 */
public final class Shooter {
  private ShooterSubsystem m_shooterSubsystem;
  public Shooter(ShooterSubsystem shoot) { m_shooterSubsystem = shoot; }

  public class Run extends InstantCommand {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

    public Run() { addRequirements(m_shooterSubsystem); }

    @Override
    public void initialize() { m_shooterSubsystem.m_flyWheel.set(0.4);}
  }

  public class Stop extends InstantCommand {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

    public Stop() { addRequirements(m_shooterSubsystem); }

    @Override
    public void initialize() { m_shooterSubsystem.m_flyWheel.set(0.0); }
  }
}
