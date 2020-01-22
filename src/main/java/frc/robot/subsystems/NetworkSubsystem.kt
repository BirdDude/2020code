/*----------------------------------------------------------------------------*/ /* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */ /* Open Source Software - may be modified and shared by FRC teams. The code   */ /* must be accompanied by the FIRST BSD license file in the root directory of */ /* the project.                                                               */ /*----------------------------------------------------------------------------*/
package frc.robot.subsystems

import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Logic.VisionComms

class NetworkSubsystem : SubsystemBase() {

    lateinit var cam1: VisionComms


    override fun periodic() {

    }

    init {
        cam1 = VisionComms("tegra-ubuntu.local", 9001)
    }

    fun bootup() {
        cam1.startUp()
    }

    fun getData() {

    }
}