/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Drivetrain

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.*

/**
 * An example command that uses an example subsystem.
 */

class DefaultDrive(private val m_DriveSubsystem: DriveSubsystem, private val m_joystickSubsystem: JoystickSubsystem, private val m_xboxSubsystem: XboxSubsystem) : CommandBase() {

    /**
 * Creates a new ExampleCommand.
 *
 * subsystem The subsystem used by this command.
 */

    var maxVel = 0.0
    var maxAcc = 0.0
    var end = false

    init {
        // Use addRequirements() here to declare subsyste m dependencies.
        addRequirements(m_DriveSubsystem)
    }

    // Called when the command is initially scheduled.
    override fun initialize() {
        end = false
    }

    // Called every time the scheduler runs while the command is scheduled.
    override fun execute() {

        maxVel = Math.max(m_DriveSubsystem.getWheelSpeeds().frontLeftMetersPerSecond, maxVel)
        maxAcc = Math.max(m_DriveSubsystem.getWheelAcc(), maxAcc)

        //FLIGHTSTICK
        m_DriveSubsystem.driveCartesan(m_joystickSubsystem.joystick.x, m_joystickSubsystem.joystick.y, m_joystickSubsystem.joystick.twist)

        //XBOX
//        m_DriveSubsystem.driveCartesan(m_xboxSubsystem.xboxController.getX(GenericHID.Hand.kLeft), m_xboxSubsystem.xboxController.getY(GenericHID.Hand.kLeft), m_xboxSubsystem.xboxController.getX(GenericHID.Hand.kRight))
    }

    // Called once the command ends or is interrupted.
    override fun end(interrupted: Boolean) {
        m_DriveSubsystem.driveCartesan(0.0, 0.0, 0.0)
        end = false
    }

    // Returns true when the command should end.
    override fun isFinished(): Boolean {
        return end
    }
}
