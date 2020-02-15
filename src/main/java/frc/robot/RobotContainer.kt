/*----------------------------------------------------------------------------*/ /* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */ /* Open Source Software - may be modified and shared by FRC teams. The code   */ /* must be accompanied by the FIRST BSD license file in the root directory of */ /* the project.                                                               */ /*----------------------------------------------------------------------------*/
package frc.robot

import edu.wpi.first.wpilibj.Joystick
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.MecanumControllerCommand
import edu.wpi.first.wpilibj.kinematics.MecanumDriveWheelSpeeds
import edu.wpi.first.wpilibj.trajectory.Trajectory
import edu.wpi.first.wpilibj.controller.PIDController
import edu.wpi.first.wpilibj.controller.ProfiledPIDController
import edu.wpi.first.wpilibj.geometry.Pose2d
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj2.command.button.JoystickButton
import frc.robot.commands.Drivetrain.DefaultDrive
import frc.robot.commands.Drivetrain.DefaultDrivePID
import frc.robot.subsystems.*


/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the [
 * Robot]
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
class RobotContainer {

    /** Subsystems*/
    private val m_driveSubsystem = DriveSubsystem()
    private val m_joystickSubsystem = JoystickSubsystem()
    private val m_xboxSubsystem = XboxSubsystem()
//    private val m_intakeSubsystem = IntakeSubsystem()

//    private val m_LidarSubsystem = LidarSubsystem()
//    private val m_visionSubsystem = VisionSubsystem()
//    private val m_controlPanelSubsystem = SpinSubsystem()

//    private val m_climberSubsystem = ClimberSubsystem()

    /**Commands */
//    private val m_defaultDrive = DefaultDrivePID(m_joystickSubsystem, m_driveSubsystem)
    private val m_defaultDrive = DefaultDrive(m_driveSubsystem, m_joystickSubsystem, m_xboxSubsystem)
    private val joystick: Joystick



    /**
     * Use this method to define your button->command mappings.  Buttons can be created by
     * instantiating a GenericHID or one of its subclasses, and then passing it to a
     * [edu.wpi.first.wpilibj2.command.button.JoystickButton].
     */
    private fun configureButtonBindings() {

        //Winch
//        JoystickButton(joystick, 12).whenPressed(Runnable { m_climberSubsystem.m_winch.set(0.3) }).whenReleased(Runnable { m_climberSubsystem.m_winch.set(0.0) })

        //Climber
//        JoystickButton(joystick, 4).whenPressed(Runnable { m_climberSubsystem.m_lifter.set(0.3) }).whenReleased(Runnable { m_climberSubsystem.m_lifter.set(0.0) })
//        JoystickButton(joystick, 6).whenPressed(Runnable { m_climberSubsystem.m_lifter.set(-0.7) }).whenReleased(Runnable { m_climberSubsystem.m_lifter.set(0.0) })

        //Intake
//        JoystickButton(joystick, 1).whenPressed(Runnable { m_intakeSubsystem.m_intake.set(0.2) }).whenReleased(Runnable { m_intakeSubsystem.m_intake.set(0.0) })

//        JoystickButton(joystick, 2).whenPressed(Runnable { m_intakeSubsystem.m_intakeDeploy.set(1.0) }).whenReleased(Runnable { m_intakeSubsystem.m_intakeDeploy.set(0.0) })
//          JoystickButton(joystick, 3).whenPressed(Runnable {
//              while(true) {
//                  m_intakeSubsystem.m_intakeDeploy.set(joystick.throttle * 0.6)
//              }
//          }).whenReleased(Runnable {
//              m_intakeSubsystem.m_intakeDeploy.set(0.0)
//          })


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


//        JoystickButton(joystick, 1).whenReleased(Runnable { m_climberSubsystem.m_rotor.set(0.3) }).whenReleased(Runnable { m_climberSubsystem.m_rotor.set(0.0) })

//        JoystickButton(joystick, 1).whenPressed( Runnable { println(m_LidarSubsystem.getLidar().getDistance())})
//        JoystickButton(joystick, 1).whenPressed(RotateTo((m_driveSubsystem.gyro.angle + 180.0) % 360, m_driveSubsystem, m_visionSubsystem).withTimeout(2.0)).whenReleased(m_defaultDrive)
//        JoystickButton(joystick, 11).whenReleased(Runnable { m_driveSubsystem.gyro.reset() })

//        JoystickButton(joystick, 1).whenPressed( )

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

    }

    fun getCartesianDrive():Command {
        return m_defaultDrive
    }
/**
    fun generatePathfindingCommand(trajectory: Trajectory): Command {
        var command = MecanumControllerCommand(
            trajectory,
            Supplier<Pose2d> {m_driveSubsystem.getMPose()},
            m_driveSubsystem.kDriveKinematics,
            PIDController(Constants.xP, Constants.xI, Constants.xD),
            PIDController(Constants.yP, Constants.yI, Constants.yD),
            ProfiledPIDController(Constants.tP, Constants.tI, Constants.tD, TrapezoidProfile.Constraints(Constants.maxRotVel, Constants.maxRotAcc)),
            Constants.forwardMaxVel,
            Consumer<MecanumDriveWheelSpeeds> { m_driveSubsystem.setManualWheelSpeeds(MecanumDriveWheelSpeeds)},
            m_driveSubsystem
            )

         return command
    }
    */
}