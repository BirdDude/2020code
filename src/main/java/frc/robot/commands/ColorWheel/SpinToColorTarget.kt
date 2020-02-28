/*----------------------------------------------------------------------------*/ /* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */ /* Open Source Software - may be modified and shared by FRC teams. The code   */ /* must be accompanied by the FIRST BSD license file in the root directory of */ /* the project.                                                               */ /*----------------------------------------------------------------------------*/
package frc.robot.commands.ColorWheel

import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.ColorWheel.SpinSubsystem

/**
 * An example command that uses an example subsystem.
 */
class SpinToColorTarget(val m_SpinSubsystem: SpinSubsystem) : CommandBase() {
    private val m_colorTarget = DriverStation.getInstance().gameSpecificMessage
    private val hasColor = m_colorTarget.equals("Y") || m_colorTarget.equals("B") || m_colorTarget.equals("G") || m_colorTarget.equals("R")

    init {
        addRequirements(m_SpinSubsystem)
    }


    var spinPos = true
    var currentPos = m_SpinSubsystem.getNearestColor()

    override fun initialize() {
        if (!hasColor) {
            println("No Color Received!")
            cancel()
        }
        else {
            println("Starting to Spin!")
            if (currentPos.equals("Blue") && m_colorTarget.equals("Y")) spinPos = true
            else if (currentPos.equals("Green") && m_colorTarget.equals("B")) spinPos = true
            else if (currentPos.equals("Red") && m_colorTarget.equals("G")) spinPos = true
            else if (currentPos.equals("Yellow") && m_colorTarget.equals("R")) spinPos = true

            if (spinPos) {
                m_SpinSubsystem.rotatorMotor.set(0.66)
            } else {
                m_SpinSubsystem.rotatorMotor.set(-0.66)
            }
        }
    }

    override fun execute() {
    }

    override fun end(interrupted: Boolean) {
        m_SpinSubsystem.rotatorMotor.set(0.0)
        println("Done Spinning!")
    }

    override fun isFinished(): Boolean {
        return m_SpinSubsystem.getNearestColor().equals(m_colorTarget)
    }

}