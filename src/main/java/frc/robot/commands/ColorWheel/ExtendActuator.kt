/*----------------------------------------------------------------------------*/ /* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */ /* Open Source Software - may be modified and shared by FRC teams. The code   */ /* must be accompanied by the FIRST BSD license file in the root directory of */ /* the project.                                                               */ /*----------------------------------------------------------------------------*/
package frc.robot.commands.ColorWheel

import edu.wpi.first.wpilibj2.command.CommandBase
import edu.wpi.first.wpilibj2.command.InstantCommand
import frc.robot.ExampleClasses.ExampleSubsystem
import frc.robot.subsystems.SpinSubsystem

/**
 * An example command that uses an example subsystem.
 */
class ExtendActuator(private val m_spinSubsystem: SpinSubsystem) : InstantCommand() {

    override fun initialize() {
        m_spinSubsystem.moveActuatorOut()
    }

    /**
     * Creates a new ExampleCommand.
     *
     * param subsystem The subsystem used by this command.
     */
    init {
        addRequirements(m_spinSubsystem)
    }
}