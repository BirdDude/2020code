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
import edu.wpi.first.wpilibj.CounterBase
import edu.wpi.first.wpilibj.Encoder
import edu.wpi.first.wpilibj.drive.MecanumDrive
import frc.robot.Constants


/**
 * Creates a new ExampleSubsystem.
 */
class DriveSubsystem : SubsystemBase() {



    var frontLeftMotor = WPI_VictorSPX(Constants.frontLeftWheelPort)
    var frontRightMotor = WPI_VictorSPX(Constants.frontRightWheelPort)
    var backLeftMotor = WPI_VictorSPX(Constants.backLeftWheelPort)
    var backRightMotor = WPI_VictorSPX(Constants.backRightWheelPort)

    var mecanum = MecanumDrive(frontLeftMotor,backLeftMotor,frontRightMotor,backRightMotor)

    var encodereFrontLeft = Encoder(0, 0, false, CounterBase.EncodingType.k2X)
    var encoderFrontRight = Encoder(0, 0, false, CounterBase.EncodingType.k2X)
    var encodereBackLeft  = Encoder(0, 0, false, CounterBase.EncodingType.k2X)
    var encoderBackRight  = Encoder(0, 0, false, CounterBase.EncodingType.k2X)


    fun driveCartesan(x: Double, y: Double, rotation: Double) {
        var trueX = x
        var trueY = y
        var trueR = rotation

        if (Math.abs(trueX) <= 0.2) trueX = 0.0
        if (Math.abs(trueY) <= 0.2) trueY = 0.0
        if (Math.abs(trueR) <= 0.5) trueR = 0.0


        if(trueR > 0) mecanum.driveCartesian(trueX, -trueY, Math.pow(trueR, 2.0))
        else mecanum.driveCartesian(trueX, -trueY, Math.pow(trueR, 2.0)*-1)
    }

    fun DriveSubsystem() {

    }

    override fun periodic() {
        // This method will be called once per scheduler run

    }
}
