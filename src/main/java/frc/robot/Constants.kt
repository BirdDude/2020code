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
    val frontRightWheelPort =   1
    val frontLeftWheelPort =    2
    val backRightWheelPort =    3
    val backLeftWheelPort =     4

    //Inputs
    val xboxPort = 0
    val joystickPort = 1

    /**Pathfinding */
    //DATA
        val isGyroReversed = false
        val dt = 0.05
        val maxVel = 0.0
        val maxAcc = 0.0
        val maxRotVel = 0.0
        val maxRotAcc = 0.0
        val maxWheelVel = 0.0
        val startPosLong = 5.0
        val startPosShort = 13.5
        val sVolts = -1.0
        val vVolts = -1.0
        val aVolts = -1.0

    //PID
        //Horizontal Travel
        val xP = 0.0
        val xI = 0.0
        val xD = 0.0
        //Vertical Travel
        val yP = 0.0
        val yI = 0.0
        val yD = 0.0
        //Rotation
        val tP = 0.0
        val tI = 0.0
        val tD = 0.0
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
