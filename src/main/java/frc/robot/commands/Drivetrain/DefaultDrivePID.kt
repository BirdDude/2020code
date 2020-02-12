/*----------------------------------------------------------------------------*/ /* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */ /* Open Source Software - may be modified and shared by FRC teams. The code   */ /* must be accompanied by the FIRST BSD license file in the root directory of */ /* the project.                                                               */ /*----------------------------------------------------------------------------*/
package frc.robot.commands.Drivetrain

import edu.wpi.first.wpilibj.Joystick
import edu.wpi.first.wpilibj.controller.PIDController
import edu.wpi.first.wpilibj.geometry.Rotation2d
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.CommandBase
import edu.wpi.first.wpilibj2.command.PIDSubsystem
import frc.robot.Constants
import frc.robot.ExampleClasses.ExampleSubsystem
import frc.robot.subsystems.DriveSubsystem

/**
 * An example command that uses an example subsystem.
 */
//public class DefaultDrivePID extends PIDCommand {
class DefaultDrivePID(joy: Joystick, drive: DriveSubsystem) : CommandBase() {

    private var m_driveSubsystem: DriveSubsystem
    lateinit var speed: ChassisSpeeds
    var joystick: Joystick
    var end = false

    var PIDX = PIDController(0.0, 0.0, 0.0)
    var PIDY = PIDController(0.0, 0.0, 0.0)
    var PIDT = PIDController(0.0, 0.0, 0.0)



    init {
        joystick = joy
        m_driveSubsystem = drive
        updateSpeed()
        addRequirements(m_driveSubsystem)

        PIDX.setIntegratorRange(-1.0, 1.0)
        PIDY.setIntegratorRange(-1.0, 1.0)
        PIDT.setIntegratorRange(-1.0, 1.0)
    }

    override fun initialize() {
        end = false
    }

    override fun execute() {
        updateSpeed()
        m_driveSubsystem.driveCartesan(PIDX.calculate(speed.vxMetersPerSecond / Constants.forwardMaxVel, joystick.x), PIDX.calculate(speed.vyMetersPerSecond / Constants.sidewaysMaxVel, joystick.y), PIDT.calculate(speed.omegaRadiansPerSecond, joystick.twist))
    }

    override fun end(interrupted: Boolean) {
        m_driveSubsystem.driveCartesan(0.0, 0.0, 0.0)
    }

    override fun isFinished(): Boolean {
        return end
    }

    private fun updateSpeed() {
        speed = ChassisSpeeds(m_driveSubsystem.gyro.velocityX.toDouble(), m_driveSubsystem.gyro.velocityY.toDouble(), degToRad(m_driveSubsystem.getHeading()))
    }

    private fun degToRad(ang: Double): Double {
        return (ang * Math.PI)/180.0
    }

    private fun radToDeg(ang: Double): Double {
        return (ang * 180.0) / Math.PI
    }
}