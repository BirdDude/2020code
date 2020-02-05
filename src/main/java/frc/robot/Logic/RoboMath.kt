package frc.robot.Logic

import frc.robot.Constants
import java.lang.Math

object RoboMath {

    fun targetX(Angle: Double, Distance: Double): Double {
        return Math.cos(Angle)*Distance
    }

    fun targetY(Angle: Double, Distance: Double): Double {
        return Math.sin(Angle)*Distance
    }

    fun powerLevel(distance: Double): Double {
        return ((distance * 100.0/81.0) * //Distance -> m/s
                60.0 )/ 0.478778720368 / Constants.maxShooterRPM //m/s -> setValue (-1.0 to 1.0)
    }


}