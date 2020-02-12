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

    fun JoyX(): Double {
        if (Math.abs(joystick.x) > 0.2) return joystick.x
        else return 0.0
    }
    fun JoyY(): Double  {
        if (Math.abs(joystick.y) > 0.2) return joystick.y
        else return 0.0
    }
    fun JoyT(): Double {
        if (Math.abs(joystick.twist) > 0.2) return joystick.twist
        else return 0.0
    }



    init {
        joystick = joy.joystick
        m_driveSubsystem = drive
        updateSpeed()
        addRequirements(m_driveSubsystem)

//        PIDX.setIntegratorRange(-1.0, 1.0)
//        PIDY.setIntegratorRange(-1.0, 1.0)
//        PIDT.setIntegratorRange(-1.0, 1.0)
        PIDT.enableContinuousInput(-180.0, 180.0)
    }

    override fun initialize() {
        end = false
    }

    override fun execute() {
        updateSpeed()

        PIDX.setpoint = joystick.x
        PIDY.setpoint = joystick.y
        PIDT.setpoint = joystick.twist



        xNum = -1 * PIDX.calculate(m_driveSubsystem.gyro.velocityX / Constants.forwardMaxVel, JoyX())
        yNum = -1 * PIDY.calculate(m_driveSubsystem.gyro.velocityY / Constants.sidewaysMaxVel, JoyY())
        tNum = -1 * PIDT.calculate(m_driveSubsystem.gyro.rawGyroZ.toDouble() / Constants.maxRotVel, JoyT())

        if (Math.abs(xNum) < 0.2) xNum = 0.0
        if (Math.abs(yNum) < 0.2) yNum = 0.0
        if (Math.abs(tNum) < 0.2) tNum = 0.0

        m_driveSubsystem.driveCartesan(xNum, yNum, tNum)
        println("Target: " + m_driveSubsystem.gyro.velocityX / Constants.forwardMaxVel + " X Out: " + xNum + "\nTarget " + m_driveSubsystem.gyro.velocityY / Constants.sidewaysMaxVel + " Y Out: " + yNum + "\nTarget: " +  JoyT() + " T Out: " + tNum)
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