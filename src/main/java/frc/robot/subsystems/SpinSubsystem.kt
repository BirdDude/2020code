/*----------------------------------------------------------------------------*/ /* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */ /* Open Source Software - may be modified and shared by FRC teams. The code   */ /* must be accompanied by the FIRST BSD license file in the root directory of */ /* the project.                                                               */ /*----------------------------------------------------------------------------*/
package frc.robot.subsystems

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX
import com.revrobotics.ColorSensorV3
import edu.wpi.first.wpilibj.I2C
import edu.wpi.first.wpilibj.Servo
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants
import java.awt.Color

class SpinSubsystem : SubsystemBase() {

    var rotationActuator = Servo(Constants.rotationActuatorPort)
    var rotator = WPI_TalonSRX(Constants.rotatorPort)
    var colorSensor = ColorSensorV3(I2C.Port.kOnboard)
    var color = colorSensor.color

    override fun periodic() {
        // This method will be called once per scheduler run
        updateColor()
    }

    fun moveActuatorIn() { rotationActuator.position = 0.17 }
    fun moveActuatorOut() { rotationActuator.position = 0.82 }

    fun isRed(): Boolean    { return (color.red/255 > 0.8) && (color.green/255 < 0.3) && (color.blue/255 < 0.3) }
    fun isGreen(): Boolean  { return (color.red/255 < 0.3) && (color.green/255 > 0.8) && (color.blue/255 < 0.3) }
    fun isBlue(): Boolean   { return (color.red/255 < 0.3) && (color.green/255 < 0.3) && (color.blue/255 > 0.8) }
    fun isYellow(): Boolean { return (color.red/255 > 0.8) && (color.green/255 > 0.8) && (color.blue/255 < 0.3) }

    fun updateColor() {
        color = colorSensor.color
    }



    /**
     * Creates a new ExampleSubsystem.
     */
    init {
        rotationActuator.setBounds(2.0, 1.8, 1.5, 1.2, 1.0);

    }
}