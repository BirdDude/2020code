/*----------------------------------------------------------------------------*/ /* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */ /* Open Source Software - may be modified and shared by FRC teams. The code   */ /* must be accompanied by the FIRST BSD license file in the root directory of */ /* the project.                                                               */ /*----------------------------------------------------------------------------*/
package frc.robot.subsystems

import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants
import frc.robot.Logic.VisionComms

class VisionSubsystem : SubsystemBase() {
    val m_visionComms = VisionComms(Constants.visionHost, Constants.visionPort)

    var loadingBearing = 0.0
    var powerPortBearing = 0.0


    override fun periodic() {
        if (m_visionComms.active) {
            try {
                if (m_visionComms.retrieveData()["pprb"] != null) powerPortBearing = m_visionComms.retrieveData()["pprb"]!!
            } catch (e: Exception) { powerPortBearing = 0.0}
            try {
                if (m_visionComms.retrieveData()["lbrb"] != null) loadingBearing = m_visionComms.retrieveData()["lbrb"]!!
            } catch (e: Exception) { loadingBearing = 0.0 }
        }

    }


    fun startUp() {
        m_visionComms.startUp()
    }

    fun shutDown() {
        m_visionComms.shutDown()
    }

    /**
     * Creates a new ExampleSubsystem.
     */
    init {

    }
}