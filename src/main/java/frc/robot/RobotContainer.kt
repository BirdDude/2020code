/*----------------------------------------------------------------------------*/ /* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */ /* Open Source Software - may be modified and shared by FRC teams. The code   */ /* must be accompanied by the FIRST BSD license file in the root directory of */ /* the project.                                                               */ /*----------------------------------------------------------------------------*/
package frc.robot

import edu.wpi.first.wpilibj.Joystick
import edu.wpi.first.wpilibj2.command.*
import edu.wpi.first.wpilibj2.command.button.JoystickButton
import frc.robot.commands.Drivetrain.DefaultDrive
import frc.robot.subsystems.*
import frc.robot.commands.Climber.Lifter
import frc.robot.commands.Climber.Winch
import frc.robot.commands.ColorWheel.Actuator
import frc.robot.commands.PowerCells.Intake
import frc.robot.commands.PowerCells.RotateIntakeBarTo
import frc.robot.commands.PowerCells.Shooter


/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the [
 * Robot]
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
class RobotContainer {

    /** Subsystems*/
    private val m_joystickSubsystem = JoystickSubsystem()
    private val m_xboxSubsystem = XboxSubsystem()


    private val m_driveSubsystem = DriveSubsystem()
    private val m_intakeSubsystem = IntakeSubsystem()
//    private val m_controlPanelSubsystem = SpinSubsystem()
    private val m_climberSubsystem = ClimberSubsystem()
//    private val m_LidarSubsystem = LidarSubsystem()
    private val m_visionSubsystem = VisionSubsystem()
    private val m_shooterSubsystem = ShooterSubsystem()

    /**Commands */
//    private val m_defaultDrive = DefaultDrivePID(m_joystickSubsystem, m_driveSubsystem)
    private val m_defaultDrive = DefaultDrive(m_driveSubsystem, m_joystickSubsystem, m_xboxSubsystem)

    private val Lifter = Lifter(m_climberSubsystem)
    private val Winch = Winch(m_climberSubsystem)
//    private val Actuator = Actuator(m_controlPanelSubsystem )
    private val Intake = Intake(m_intakeSubsystem)
    private val Shooter = Shooter(m_shooterSubsystem)


    private val joystick: Joystick



    /**
     * Use this method to define your button->command mappings.  Buttons can be created by
     * instantiating a GenericHID or one of its subclasses, and then passing it to a
     * [edu.wpi.first.wpilibj2.command.button.JoystickButton].
     */
    private fun configureButtonBindings() {

//        Winch
//        JoystickButton(joystick, 12).whenPressed(Winch.Run()).whenReleased(Winch.Stop())

//        Climber
//        JoystickButton(joystick, 4).whenPressed(Lifter.Raise()).whenReleased(Lifter.Stop())
//        JoystickButton(joystick, 6).whenPressed(Lifter.Lower()).whenReleased(Lifter.Stop())

        //Intake
//        JoystickButton(joystick, 2).whenPressed(RotateIntakeBarTo(Constants.downPosEncoderTicks, m_intakeSubsystem).andThen(Intake.Run()))
//                .whenReleased(Intake.Stop().andThen(RotateIntakeBarTo(0.0, m_intakeSubsystem)))
//        JoystickButton(joystick, 11).whenPressed(Runnable { println(m_intakeSubsystem.m_intakeDeploy.selectedSensorPosition) })
        JoystickButton(joystick, 11).whenPressed(Shooter.Run()).whenReleased(Shooter.Stop())

//        ColorWheel extending system
//        var is11Active = false
//        JoystickButton(joystick, -1).whenPressed(Runnable {
//            if (is11Active) {
//                is11Active = false
//                ExtendActuator(m_rotatorSubsystem)
//            } else {
//                is11Active = true
//                RetractActuator(m_rotatorSubsystem)
//            }
//        })


        /** Testing */

//        JoystickButton(joystick, 11).whenPressed(Runnable { println(m_visionSubsystem.startUp()) })
//        JoystickButton(joystick, 12).whenPressed(Runnable { println(m_visionSubsystem.shutDown()) })
//        JoystickButton(joystick, 1).whenPressed(Runnable { println("PP Bearing: " + m_visionSubsystem.powerPortBearing + "\nLB Bearing: " + m_visionSubsystem.loadingBearing) })
//        JoystickButton(joystick, 11).whenPressed(SpinToColorTarget(m_rotatorSubsystem, "Blue"))

//        JoystickButton(joystick, 11).whenPressed(ExtendActuator(m_controlPanelSubsystem)).whenReleased(RetractActuator(m_controlPanelSubsystem))



//        JoystickButton(joystick, 1).whenPressed( Runnable { println(m_LidarSubsystem.getLidar().getDistance())})

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

        //Jetson Bootup
//        BootJetson().schedule()

        joystick = m_joystickSubsystem.joystick


        // Configure the button bindings

        configureButtonBindings()
        m_defaultDrive.initialize()

    }

    fun getCartesianDrive():Command {
        return m_defaultDrive
    }

/**
    fun generatePathfindingCommand(trajectory: Trajectory): Command {
        var command = MecanumControllerCommand(
                trajectory,
                Supplier { m_driveSubsystem.getMPose() },
                SimpleMotorFeedforward(0.0, 0.0, 0.0)
        )

        return command
    }
 */

}
