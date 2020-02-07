/*----------------------------------------------------------------------------*/ /* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */ /* Open Source Software - may be modified and shared by FRC teams. The code   */ /* must be accompanied by the FIRST BSD license file in the root directory of */ /* the project.                                                               */ /*----------------------------------------------------------------------------*/
package frc.robot.subsystems

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX
import com.revrobotics.ColorMatch
import com.revrobotics.ColorSensorV3
import edu.wpi.first.wpilibj.I2C
import edu.wpi.first.wpilibj.Servo
import edu.wpi.first.wpilibj.util.Color
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants


class SpinSubsystem : SubsystemBase() {

    var rotationActuator = Servo(Constants.rotationActuatorPort)
    var rotatorMotor = WPI_TalonSRX(Constants.rotatorPort)
    var colorSensor = ColorSensorV3(I2C.Port.kOnboard)
    private var color = colorSensor.color
    private val m_colorMatcher = ColorMatch()
    private val kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429)
    private val kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240)
    private val kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114)
    private val kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113)


    override fun periodic() { }

    fun moveActuatorIn() { rotationActuator.position = 0.17 }
    fun moveActuatorOut() { rotationActuator.position = 0.82 }

    fun getColor(): Color {
        color = colorSensor.color
        return color
    }

    fun getNearestColor(): String {
        color = colorSensor.color

        val match = m_colorMatcher.matchClosestColor(color)

        if (match.color === kBlueTarget) {
            return "Blue"
        } else if (match.color === kRedTarget) {
            return "Red"
        } else if (match.color === kGreenTarget) {
            return "Green"
        } else if (match.color === kYellowTarget) {
            return "Yellow"
        } else {
            return "Unknown"
        }
    }


    /**
     * Creates a new ExampleSubsystem.
     */
    init {
        rotationActuator.setBounds(2.0, 1.8, 1.5, 1.2, 1.0)

        m_colorMatcher.addColorMatch(kBlueTarget)
        m_colorMatcher.addColorMatch(kGreenTarget)
        m_colorMatcher.addColorMatch(kRedTarget)
        m_colorMatcher.addColorMatch(kYellowTarget)

    }
}