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

    //Info (Meters)
    val wheelRad = 0.0762
    val wheelCircum = 2 * Math.PI * wheelRad


    //MotorControllers
    const val frontLeftWheelPort =    1
    const val frontRightWheelPort =   2
    const val backLeftWheelPort =     3
    const val backRightWheelPort =    4


    //Inputs
    const val xboxPort = 0
    const val joystickPort = 1

    /**Pathfinding */
    //DATA
    const val isGyroReversed = false
    const val dt = 0.05
    const val maxVel = 0.0
    const val maxAcc = 0.0
    const val maxRotVel = 0.0
    const val maxRotAcc = 0.0
    const val maxWheelVel = 0.0
    const val startPosLong = 5.0
    const val startPosShort = 13.5
    const val sVolts = -1.0
    const val vVolts = -1.0
    const val aVolts = -1.0

    //PID
        //Horizontal Travel
    const val xP = 0.0
    const val xI = 0.0
    const val xD = 0.0
        //Vertical Travel
    const val yP = 0.0
    const val yI = 0.0
    const val yD = 0.0
        //Rotation
    const val tP = 0.0
    const val tI = 0.0
    const val tD = 0.0
    /**
        //FrontLeft Wheel (Maybe Unneeded)
        val flP = 0.0
        val flI = 0.0
        val flD = 0.0
        //FrontRight Wheel (Maybe Unneeded)
        val frP = 0.0
        val frI = 0.0
        val frD = 0.0
        //BackLeft Wheel (Maybe Unneeded)
        val blP = 0.0
        val blI = 0.0
        val bD = 0.0
        //BackRight Wheel (Maybe Unneeded)
        val brP = 0.0
        val brI = 0.0
        val brD = 0.0
        */




    //Data Inputs
//    val gyroPort = -1


}
