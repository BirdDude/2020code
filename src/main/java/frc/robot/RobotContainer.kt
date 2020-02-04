/*----------------------------------------------------------------------------*/ /* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */ /* Open Source Software - may be modified and shared by FRC teams. The code   */ /* must be accompanied by the FIRST BSD license file in the root directory of */ /* the project.                                                               */ /*----------------------------------------------------------------------------*/
package frc.robot

import edu.wpi.cscore.CameraServerJNI
import edu.wpi.cscore.VideoMode
import edu.wpi.first.cameraserver.CameraServer
import edu.wpi.first.wpilibj.Joystick
import edu.wpi.first.wpilibj.Servo
import edu.wpi.first.wpilibj.controller.PIDController
import edu.wpi.first.wpilibj.controller.ProfiledPIDController
import edu.wpi.first.wpilibj.geometry.Pose2d
import edu.wpi.first.wpilibj.geometry.Rotation2d
import edu.wpi.first.wpilibj.geometry.Translation2d
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.MecanumControllerCommand
import edu.wpi.first.wpilibj2.command.button.JoystickButton
import frc.robot.Logic.RoboMath
import frc.robot.commands.DefaultDrive
import frc.robot.commands.PathfindTo
import frc.robot.commands.RotateTo
import frc.robot.subsystems.*
import java.nio.channels.Channel
import java.util.function.Consumer
import java.util.function.Supplier

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the [
 * Robot]
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private val m_driveSubsystem = DriveSubsystem()
    private val m_joystickSubsystem = JoystickSubsystem()
    private val m_xboxSubsystem = XboxSubsystem()
    private val m_LidarSubsystem = LidarSubsystem()
    private val m_visionSubsystem = VisionSubsystem()
    private val m_rotatorSubsystem = SpinSubsystem()


    private val m_defaultDrive = DefaultDrive(m_driveSubsystem, m_joystickSubsystem, m_xboxSubsystem)
    private val joystick: Joystick
    private val m_camera = CameraServer.getInstance().startAutomaticCapture()



    /**
     * Use this method to define your button->command mappings.  Buttons can be created by
     * instantiating a GenericHID or one of its subclasses, and then passing it to a
     * [edu.wpi.first.wpilibj2.command.button.JoystickButton].
     */
    private fun configureButtonBindings() {



        //Testing
//        JoystickButton(joystick, 1).whenPressed( Runnable { println(m_LidarSubsystem.getLidar().getDistance())})

//        JoystickButton(joystick, 1).whenPressed(RotateTo((m_driveSubsystem.gyro.angle + 180.0) % 360, m_driveSubsystem, m_visionSubsystem).withTimeout(2.0)).whenReleased(m_defaultDrive)
//        JoystickButton(joystick, 11).whenReleased(Runnable { m_driveSubsystem.gyro.reset() })

//        JoystickButton(joystick, 1).whenPressed(Runnable { PathfindTo(1.0, 1.0, 90.0, m_driveSubsystem) } )

//        JoystickButton(joystick, 3).whenPressed(Runnable { m_defaultDrive.end(true) }).whenPressed(goToVisionTarget())
//        JoystickButton(joystick, 4).whenPressed(Runnable { goToVisionTarget().end(true) }).whenPressed(Runnable { if(!m_defaultDrive.isScheduled) m_defaultDrive.schedule() })

    }


    /**
     * Use this to pass the autonomous command to the main [Robot] class.
     *
     * @return the command to run in autonomous
     */

//    fun getAutonomousCommand(): Command {
//
//    }




    /**
     * The container for the robot.  Contains subsystems, OI devices, and commands.
     */


    init {
        joystick = m_joystickSubsystem.joystick

        // Configure the button bindings
        configureButtonBindings()
        m_defaultDrive.initialize()

        m_camera.setFPS(30)
        m_camera.setExposureManual(15)
        m_camera.setResolution(480, 270)
//        m_camera.setWhiteBalanceManual(4000)
        m_camera.setPixelFormat(VideoMode.PixelFormat.kMJPEG)

    }

    fun getCartesianDrive():Command {
        return m_defaultDrive
    }



}