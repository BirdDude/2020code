/*----------------------------------------------------------------------------*/ /* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */ /* Open Source Software - may be modified and shared by FRC teams. The code   */ /* must be accompanied by the FIRST BSD license file in the root directory of */ /* the project.                                                               */ /*----------------------------------------------------------------------------*/
package frc.robot.commands.Drivetrain

import edu.wpi.first.wpilibj.controller.PIDController
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.Constants
import frc.robot.subsystems.DriveSubsystem
import frc.robot.subsystems.JoystickSubsystem

/**
 * An example command that uses an example subsystem.
 */
//public class DefaultDrivePID extends PIDCommand {
class DefaultDrivePID(joy: JoystickSubsystem, drive: DriveSubsystem) : CommandBase() {

    private var m_driveSubsystem: DriveSubsystem
    lateinit var speed: ChassisSpeeds
    var joystick = joy.joystick
    var end = false

    var PIDX = PIDController(0.55, 0.0, 0.0)
    var PIDY = PIDController(0.55, 0.0, 0.0)
    var PIDT = PIDController(0.55, 0.0, 0.3)

    var xNum = 0.0
    var yNum = 0.0
    var tNum = 0.0

    private fun joyX(): Double {
        return if (Math.abs(joystick.x) > 0.2) joystick.x
        else {
            PIDX.reset()
            0.0
        }
    }
    private fun joyY(): Double  {
        return if (Math.abs(joystick.y) > 0.2) joystick.y
        else {
            PIDY.reset()
            0.0
        }
    }
    private fun joyT(): Double {
        return if (Math.abs(joystick.twist) > 0.2) joystick.twist
        else {
            PIDT.reset()
            0.0
        }
    }



    init {
        joystick = joy.joystick
        m_driveSubsystem = drive
        addRequirements(m_driveSubsystem)

        PIDT.enableContinuousInput(-180.0, 180.0)

        PIDX.setTolerance(0.1)
        PIDY.setTolerance(0.1)
        PIDT.setTolerance(0.1)

    }

    override fun initialize() {
        end = false

        PIDX.setpoint = joystick.x
        PIDY.setpoint = joystick.y
        PIDT.setpoint = joystick.twist
    }

    override fun execute() {

        xNum = -1 * PIDX.calculate(m_driveSubsystem.gyro.velocityX / Constants.forwardMaxVel, joyX())
        yNum = -1 * PIDY.calculate(m_driveSubsystem.gyro.velocityY / Constants.sidewaysMaxVel, joyY())
        tNum = -1 * PIDT.calculate(m_driveSubsystem.gyro.rawGyroZ.toDouble() / Constants.maxRotVel, joyT())

        m_driveSubsystem.driveCartesan(xNum, yNum, tNum)
        println("Target: " + m_driveSubsystem.gyro.velocityX / Constants.forwardMaxVel + " X Out: " + xNum + "\nTarget " + m_driveSubsystem.gyro.velocityY / Constants.sidewaysMaxVel + " Y Out: " + yNum + "\nTarget: " +  tNum + " T Out: " + tNum)
    }

    override fun end(interrupted: Boolean) {
        m_driveSubsystem.driveCartesan(0.0, 0.0, 0.0)
    }

    override fun isFinished(): Boolean {
        return end
    }

    private fun degToRad(ang: Double): Double {
        return (ang * Math.PI)/180.0
    }

    private fun radToDeg(ang: Double): Double {
        return (ang * 180.0) / Math.PI
    }
}