/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 *
 * It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
object Constants {

    //Todo Everything here, oh no

    //Info (Meters)
    val wheelRad = 0.0762
    val wheelCircum = 2 * Math.PI * wheelRad


    /**MotorControllers */
    //Wheels
    const val frontLeftWheelPort =    1
    const val frontRightWheelPort =   2
    const val backLeftWheelPort =     3
    const val backRightWheelPort =    4

    //Rotator
    const val rotationActuatorPort =  0
    const val rotatorPort = -1

    //Test
    const val testPort = -1

    //Intake
    const val intakePort = -1

    //Shooter
    const val shooterPort = -1

    //Transport
    const val transportPort = -1
    const val transportWheelPort = -1

    //Climb
    const val climbRotor = -1
    const val clumbWinch = -1

    //Control Panel
//    const val

    /**Inputs */
    //Controllers
    const val xboxPort = 0
    const val joystickPort = 1

    //Vision
    const val visionHost = "10.43.30.20"
    const val visionPort = 9001


    /**Pathfinding */
    //DATA
    const val isGyroReversed = false
    const val dt = 0.05
    const val maxVel = 2.8497620125792773
    const val maxAcc = 1.3895263671875


    const val maxRotVel = 0.0
    const val maxRotAcc = 0.0
    const val maxRotDeg = 0.0
    const val startPosLong = 5.0
    const val startPosShort = 13.5
    const val sVolts = 0.0
    const val vVolts = 0.0

    //PID
        //Horizontal Travel
    const val xP = 1.0
    const val xI = 0.0
    const val xD = 0.0
        //Vertical Travel
    const val yP = 1.0
    const val yI = 0.0
    const val yD = 0.0
        //Rotation
    const val tP = 5.0
    const val tI = 0.0
    const val tD = 0.5


}
