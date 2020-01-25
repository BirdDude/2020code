package frc.robot.Logic

import java.lang.Math

object RoboMath {

    fun targetX(Angle: Double, Distance: Double): Double {
        return Math.cos(Angle)*Distance
    }

    fun targetY(Angle: Double, Distance: Double): Double {
        return Math.sin(Angle)*Distance
    }


}