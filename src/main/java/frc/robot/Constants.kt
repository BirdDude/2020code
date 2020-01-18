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

    //MotorControllers
    val frontRightWheelPort =   1
    val frontLeftWheelPort =    2
    val backRightWheelPort =    3
    val backLeftWheelPort =     4

    //Inputs
    val xboxPort = 0
    val joystickPort = 1

    //Pathfinding
    val isGyroReversed = false
    val dt = 0.05
    val maxVel = 0.0
    val maxAcc = 0.0
    val maxJerk = 0.0
    val wheelBase_width = 0.0
    val wheelBase_depth = 0.0
    val kRamseteB = 2.0
    val kRamseteZeta = 0.7
    val startPosLong = 5.0
    val startPosShort = 13.5


    //Data Inputs
//    val gyroPort = -1


}
