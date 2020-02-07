/*----------------------------------------------------------------------------*/ /* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */ /* Open Source Software - may be modified and shared by FRC teams. The code   */ /* must be accompanied by the FIRST BSD license file in the root directory of */ /* the project.                                                               */ /*----------------------------------------------------------------------------*/
package frc.robot.commands.ColorWheel

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.ExampleClasses.ExampleSubsystem
import frc.robot.subsystems.SpinSubsystem

/**
 * An example command that uses an example subsystem.
 */
class SpinToColorTarget(val m_SpinSubsystem: SpinSubsystem, var m_colorTarget: String) : CommandBase() {

    var spinPos = true
    var currentPos = m_SpinSubsystem.getNearestColor()

    override fun initialize() {
        if (currentPos.equals("Blue") && m_colorTarget.equals("Yellow")) spinPos = true
        else if (currentPos.equals("Green") && m_colorTarget.equals("Blue")) spinPos = true
        else if (currentPos.equals("Red") && m_colorTarget.equals("Green")) spinPos = true
        else if (currentPos.equals("Yellow") && m_colorTarget.equals("Red")) spinPos = true
    }

    override fun execute() {
        if (spinPos) {
            m_SpinSubsystem.rotatorMotor.set(0.5)
        } else {
            m_SpinSubsystem.rotatorMotor.set(-0.5)
        }
    }

    override fun end(interrupted: Boolean) {
        m_SpinSubsystem.rotatorMotor.set(0.0)
    }

    override fun isFinished(): Boolean {
        return m_SpinSubsystem.getNearestColor().equals(m_colorTarget)
    }

    /**
     * Creates a new ExampleCommand.
     *
     * param subsystem The subsystem used by this command.
     */
    init { addRequirements(m_SpinSubsystem) }
}