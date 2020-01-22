/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands

import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.*

/**
 * An example command that uses an example subsystem.
 */

class DefaultDrive
/**
 * Creates a new ExampleCommand.
 *
 * subsystem The subsystem used by this command.
 */
(private val m_DriveSubsystem: DriveSubsystem, private val m_joystickSubsystem: JoystickSubsystem, private val m_xboxSubsystem: XboxSubsystem) : CommandBase() {

    init {
        // Use addRequirements() here to declare subsyste m dependencies.
        addRequirements(m_DriveSubsystem)
    }

    // Called when the command is initially scheduled.
    override fun initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    override fun execute() {
        //FLIGHTSTICK
        m_DriveSubsystem.driveCartesan(m_joystickSubsystem.joystick.x, m_joystickSubsystem.joystick.y, m_joystickSubsystem.joystick.twist)

        //XBOX
//        m_DriveSubsystem.driveCartesan(m_xboxSubsystem.xboxController.getX(GenericHID.Hand.kLeft), m_xboxSubsystem.xboxController.getY(GenericHID.Hand.kLeft), m_xboxSubsystem.xboxController.getX(GenericHID.Hand.kRight), m_DriveSubsystem.gyro.angle)
//        m_DriveSubsystem.driveCartesan(m_xboxSubsystem.xboxController.getX(GenericHID.Hand.kLeft), m_xboxSubsystem.xboxController.getY(GenericHID.Hand.kLeft), m_xboxSubsystem.xboxController.getX(GenericHID.Hand.kRight))
    }

    // Called once the command ends or is interrupted.
    override fun end(interrupted: Boolean) {

    }

    // Returns true when the command should end.
    override fun isFinished(): Boolean {
        return false
    }
}
