/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems

import edu.wpi.first.wpilibj2.command.SubsystemBase
import com.ctre.phoenix.motorcontrol.can.*
import edu.wpi.first.hal.FRCNetComm
import edu.wpi.first.wpilibj.drive.MecanumDrive
import frc.robot.Constants


/**
 * Creates a new ExampleSubsystem.
 */
class DriveSubsystem : SubsystemBase() {



    var frontLeftMotor = WPI_VictorSPX(Constants.frontLeftWheelPort)
    var frontRightMotor = WPI_TalonSRX(Constants.frontRightWheelPort)
    var backLeftMotor = WPI_TalonSRX(Constants.backLeftWheelPort)
    var backRightMotor = WPI_TalonSRX(Constants.backRightWheelPort)

    var mecanum = MecanumDrive(frontLeftMotor,backLeftMotor,frontRightMotor,backRightMotor)

    fun driveCartesan(x: Double, y: Double, rotation: Double) {
        mecanum.driveCartesian(x, y, rotation)
    }

    fun DriveSubsystem() {

    }

    override fun periodic() {
        // This method will be called once per scheduler run

    }
}
