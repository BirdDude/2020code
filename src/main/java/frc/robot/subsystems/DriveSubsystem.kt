/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

/**
 * For Ease of Copying and Pasting
    var frontLeft   =
    var frontRight  =
    var backLeft    =
    var backRight   =

 */

package frc.robot.subsystems

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX
import com.kauailabs.navx.frc.AHRS
import edu.wpi.first.wpilibj.SPI
import edu.wpi.first.wpilibj.drive.MecanumDrive
import edu.wpi.first.wpilibj.geometry.Pose2d
import edu.wpi.first.wpilibj.geometry.Rotation2d
import edu.wpi.first.wpilibj.geometry.Translation2d
import edu.wpi.first.wpilibj.interfaces.Accelerometer
import edu.wpi.first.wpilibj.kinematics.MecanumDriveKinematics
import edu.wpi.first.wpilibj.kinematics.MecanumDriveOdometry
import edu.wpi.first.wpilibj.kinematics.MecanumDriveWheelSpeeds
import edu.wpi.first.wpilibj.trajectory.constraint.MecanumDriveKinematicsConstraint
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants


/**
 * Creates a new ExampleSubsystem.
 */
class DriveSubsystem : SubsystemBase() {

    //Motor/Drive Init
    var frontLeftMotor = WPI_TalonSRX(Constants.frontLeftWheelPort)
    var frontRightMotor = WPI_TalonSRX(Constants.frontRightWheelPort)
    var backLeftMotor = WPI_TalonSRX(Constants.backLeftWheelPort)
    var backRightMotor = WPI_TalonSRX(Constants.backRightWheelPort)
    var mecanum = MecanumDrive(frontLeftMotor,backLeftMotor,frontRightMotor,backRightMotor)


    //Robot Bounds
    var frontRightPos = Translation2d(0.381, 0.381)
    var frontLeftPos = Translation2d(-0.381, 0.381)
    var backRightPos = Translation2d(0.381, -0.381)
    var backLeftPos = Translation2d(-0.381, -0.381)
    var kDriveKinematics = MecanumDriveKinematics(frontLeftPos, frontRightPos, backLeftPos, backRightPos)
    var kDriveConstraints = MecanumDriveKinematicsConstraint(kDriveKinematics, Constants.maxVel)


    //Gyro Init
    val gyro = AHRS(SPI.Port.kMXP)



    var m_odometry = MecanumDriveOdometry(kDriveKinematics, Rotation2d.fromDegrees(getHeading()), Pose2d(Constants.startPosLong, Constants.startPosShort, Rotation2d.fromDegrees(getHeading())))
    lateinit var m_pose: Pose2d
    lateinit var speed: () -> MecanumDriveWheelSpeeds

    init {
        //Just me here, huh?
    }




    fun driveCartesan(x: Double, y: Double, rotation: Double, angle: Double) {
        var trueX = x
        var trueY = y
        var trueR = rotation

        if (Math.abs(trueX) <= 0.2) trueX = 0.0
        if (Math.abs(trueY) <= 0.2) trueY = 0.0
        if (Math.abs(trueR) <= 0.5) trueR = 0.0


        if(trueR > 0) mecanum.driveCartesian(trueX, -trueY, Math.pow(trueR, 2.0), angle)
        else mecanum.driveCartesian(trueX, -trueY, Math.pow(trueR, 2.0)*-1, angle)
    }

    fun driveCartesan(x: Double, y: Double, rotation: Double) {
        var trueX = x
        var trueY = y
        var trueR = rotation

        if (Math.abs(trueX) <= 0.2) trueX = 0.0
        if (Math.abs(trueY) <= 0.2) trueY = 0.0
        if (Math.abs(trueR) <= 0.5) trueR = 0.0

//        println("Rotation: " + rotation)

        if(trueR > 0) mecanum.driveCartesian(trueX, -trueY, Math.pow(trueR, 2.0))
        else mecanum.driveCartesian(trueX, -trueY, Math.pow(trueR, 2.0)*-1)
    }

    fun driveCartesanRobot(x: Double, y: Double, rotation: Double) {
        println("Rotation: " + rotation)

        mecanum.driveCartesian(x, y, rotation)
    }

    fun autoCartesian(x: Double, y: Double, rotation: Double) {
        mecanum.driveCartesian(x, y, rotation)
    }



    override fun periodic() {
        // This method will be called once per scheduler run

        /** Re-add when Added Encoders */
        speed = { MecanumDriveWheelSpeeds(frontLeftMotor.selectedSensorVelocity * Constants.wheelCircum, frontRightMotor.selectedSensorVelocity * Constants.wheelCircum,
                                        backLeftMotor.selectedSensorVelocity * Constants.wheelCircum,  backRightMotor.selectedSensorVelocity * Constants.wheelCircum) }


        m_pose = m_odometry.update(Rotation2d.fromDegrees(getHeading()), speed.invoke())
    }

    fun getHeading(): Double {
//        return Math.IEEEremainder(gyro.getAngle(), 360.0) * if (Constants.isGyroReversed) -1.0 else 1.0
        return gyro.angle
    }

    fun setmManualWheelSpeeds(speed: MecanumDriveWheelSpeeds) {
        frontLeftMotor.set(speed.frontLeftMetersPerSecond)
        frontRightMotor.set(speed.frontRightMetersPerSecond)
        backLeftMotor.set(speed.rearLeftMetersPerSecond)
        backRightMotor.set(speed.rearRightMetersPerSecond)
    }

    fun getWheelSpeeds(): MecanumDriveWheelSpeeds {
        return MecanumDriveWheelSpeeds(frontLeftMotor.selectedSensorVelocity * (10.0/4096) * Constants.wheelCircum, frontRightMotor.selectedSensorVelocity  *(10.0/4096) * Constants.wheelCircum,
                backLeftMotor.selectedSensorVelocity * (10.0/4096) * Constants.wheelCircum,  backRightMotor.selectedSensorVelocity * (10.0/4096) * Constants.wheelCircum)
    }

    fun getWheelAcc(): Double {
        return gyro.rawAccelX.toDouble()
    }

    fun getM_Pose(): Pose2d {
        return m_pose
    }
}
