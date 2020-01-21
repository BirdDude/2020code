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
import frc.robot.commands.DefaultDrive
import frc.robot.commands.ExampleCommand
import frc.robot.subsystems.DriveSubsystem
import frc.robot.subsystems.ExampleSubsystem
import frc.robot.subsystems.JoystickSubsystem
import frc.robot.subsystems.XboxSubsystem
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
    private val m_exampleSubsystem = ExampleSubsystem()
    private val m_driveSubsystem = DriveSubsystem()
    private val m_joystickSubsystem = JoystickSubsystem()
    private val m_xboxSubsystem = XboxSubsystem()


    private val m_autoCommand = ExampleCommand(m_exampleSubsystem)
    private val m_defaultDrive = DefaultDrive(m_driveSubsystem, m_joystickSubsystem, m_xboxSubsystem)
    private val joystick: Joystick

    /**
     * Use this method to define your button->command mappings.  Buttons can be created by
     * instantiating a [GenericHID] or one of its subclasses ([ ] or [XboxController]), and then passing it to a
     * [edu.wpi.first.wpilibj2.command.button.JoystickButton].
     */
    private fun configureButtonBindings() {
//        JoystickButton trigger = new JoystickButton(joystick, 1);  FOR FUTURE REFRENCE
//        trigger.whenPressed(new DefaultDrive());


    }

//    An ExampleCommand will run in autonomous

    /**
     * Use this to pass the autonomous command to the main [Robot] class.
     *
     * @return the command to run in autonomous
     */


    val followTrajectoryCommand: Command
        get() {
            val config = TrajectoryConfig(Constants.maxVel, Constants.maxAcc).setKinematics(m_driveSubsystem.kDriveKinematics).addConstraint(m_driveSubsystem.kDriveConstraints)

            val exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                    Pose2d(0.0, 0.0, Rotation2d(0.0)),
                    mutableListOf(Translation2d(1.0, 1.0), Translation2d(2.0, -1.0)), Pose2d(0.0, 0.0, Rotation2d(0.0)), config)

            val command = MecanumControllerCommand(
                    exampleTrajectory,
                    Supplier {m_driveSubsystem.m_pose},
//                    m_pathfinderCommand.getFeedForward(),
                    m_driveSubsystem.kDriveKinematics,
                    PIDController(Constants.xP, Constants.xI, Constants.xD, Constants.dt),
                    PIDController(Constants.yP, Constants.yI, Constants.yD, Constants.dt),
                    ProfiledPIDController(Constants.tP, Constants.tI, Constants.tD, TrapezoidProfile.Constraints(Constants.maxRotVel, Constants.maxRotAcc), Constants.dt),
                    Constants.maxWheelVel,
                    Consumer { m_driveSubsystem.getWheelSpeeds() },
//                    driveSubsystem::setVoltage,
                    m_driveSubsystem
            )
            return command.andThen(
                    Runnable { m_driveSubsystem.driveCartesan(0.0, 0.0, 0.0, 0.0); })
        }

    /**
     * The container for the robot.  Contains subsystems, OI devices, and commands.
     */
    init {
        joystick = m_joystickSubsystem.joystick
        // Configure the button bindings
        configureButtonBindings()
        m_defaultDrive.initialize()
    }
}