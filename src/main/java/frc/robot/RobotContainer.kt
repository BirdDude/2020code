/*----------------------------------------------------------------------------*/ /* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */ /* Open Source Software - may be modified and shared by FRC teams. The code   */ /* must be accompanied by the FIRST BSD license file in the root directory of */ /* the project.                                                               */ /*----------------------------------------------------------------------------*/
package frc.robot

import edu.wpi.first.wpilibj.Joystick
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
import edu.wpi.first.wpilibj2.command.PrintCommand
import edu.wpi.first.wpilibj2.command.button.JoystickButton
import frc.robot.Logic.RoboMath
import frc.robot.Logic.VisionComms
import frc.robot.commands.DefaultDrive
import frc.robot.commands.RotateTo
import frc.robot.subsystems.*
import java.util.function.Consumer
import java.util.function.Supplier

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the [Robot]
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private val m_driveSubsystem = DriveSubsystem()
    private val m_joystickSubsystem = JoystickSubsystem()
    private val m_xboxSubsystem = XboxSubsystem()
    private val m_LidarSubsystem = LidarSubsystem()
    private val m_rotationSubsystem = RotationSubsystem(m_driveSubsystem)


    private val m_defaultDrive = DefaultDrive(m_driveSubsystem, m_joystickSubsystem, m_xboxSubsystem)
    private val m_rotateTo = RotateTo(m_rotationSubsystem, m_driveSubsystem)
    private val joystick: Joystick
    private val m_visionComms = VisionComms(Constants.visionHost, Constants.visionPort)


    /**
     * Use this method to define your button->command mappings.  Buttons can be created by
     * instantiating a [GenericHID] or one of its subclasses ([ ] or [XboxController]), and then passing it to a
     * [edu.wpi.first.wpilibj2.command.button.JoystickButton].
     */
    private fun configureButtonBindings() {
//        JoystickButton trigger = new JoystickButton(joystick, 1);  FOR FUTURE REFRENCE
//        trigger.whenPressed(new DefaultDrive());

        JoystickButton(joystick, 11).whenPressed(Runnable { m_visionComms.startUp()  })
        JoystickButton(joystick, 12).whenPressed(Runnable { m_visionComms.shutDown() })

        JoystickButton(joystick, 1).whenPressed(Runnable { println("Variables: " + m_visionComms.retrieveData())})
    }

//    An ExampleCommand will run in autonomous

    /**
     * Use this to pass the autonomous command to the main [Robot] class.
     *
     * @return the command to run in autonomous
     */

//    fun getAutonomousCommand(): Command {
//
//    }


    fun goToVisionTarget(): Command {


        val startX = 0.0
        val startY = 0.0
        val startR = 0.0

        //Assuming looking straight at target
        val endX = RoboMath.targetX(m_driveSubsystem.getHeading(), m_LidarSubsystem.getLidar().getDistance().toDouble())
        val endY = RoboMath.targetY(m_driveSubsystem.getHeading(), m_LidarSubsystem.getLidar().getDistance().toDouble())
        val endR = -90.0

            val config = TrajectoryConfig(Constants.maxVel, Constants.maxAcc).setKinematics(m_driveSubsystem.kDriveKinematics).addConstraint(m_driveSubsystem.kDriveConstraints)

            val exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                    Pose2d(startX, startY, Rotation2d(startR)),
                    mutableListOf(), Pose2d(Translation2d(endX, endY), Rotation2d(endR)), config)

            val command = MecanumControllerCommand(
                    exampleTrajectory,
                    Supplier {m_driveSubsystem.m_pose},
                    m_driveSubsystem.kDriveKinematics,
                    PIDController(Constants.xP, Constants.xI, Constants.xD, Constants.dt),
                    PIDController(Constants.yP, Constants.yI, Constants.yD, Constants.dt),
                    ProfiledPIDController(Constants.tP, Constants.tI, Constants.tD, TrapezoidProfile.Constraints(Constants.maxRotVel, Constants.maxRotAcc), Constants.dt),
                    Constants.maxWheelVel,
                    Consumer { m_driveSubsystem.getWheelSpeeds() },
                    m_driveSubsystem
            )
            return command.andThen(Runnable { m_driveSubsystem.driveCartesan(0.0, 0.0, 0.0) })
        }

    /**
     * The container for the robot.  Contains subsystems, OI devices, and commands.
     */
    init {
        joystick = m_joystickSubsystem.joystick

        // Configure the button bindings
        configureButtonBindings()
//        m_defaultDrive.initialize()
    }

    fun getCartesianDrive():Command {
        return m_defaultDrive
    }

}