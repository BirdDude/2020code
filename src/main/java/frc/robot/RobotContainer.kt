/*----------------------------------------------------------------------------*/ /* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */ /* Open Source Software - may be modified and shared by FRC teams. The code   */ /* must be accompanied by the FIRST BSD license file in the root directory of */ /* the project.                                                               */ /*----------------------------------------------------------------------------*/
package frc.robot

import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Joystick
import edu.wpi.first.wpilibj.controller.PIDController
import edu.wpi.first.wpilibj.controller.ProfiledPIDController
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward
import edu.wpi.first.wpilibj.geometry.Pose2d
import edu.wpi.first.wpilibj.geometry.Rotation2d
import edu.wpi.first.wpilibj.geometry.Translation2d
import edu.wpi.first.wpilibj.trajectory.Trajectory
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.ConditionalCommand
import edu.wpi.first.wpilibj2.command.MecanumControllerCommand
import edu.wpi.first.wpilibj2.command.WaitCommand
import edu.wpi.first.wpilibj2.command.button.JoystickButton
import frc.robot.commands.Autonomous.DriveToPowerPort
import frc.robot.commands.Autonomous.ForceDrive
import frc.robot.commands.Climber.Lifter
import frc.robot.commands.Climber.Winch
import frc.robot.commands.Drivetrain.DefaultDrive
import frc.robot.commands.Drivetrain.RotateTo
import frc.robot.commands.PowerCells.Intake
import frc.robot.commands.PowerCells.RotateIntakeBarTo
import frc.robot.commands.PowerCells.Shooter
import frc.robot.commands.PowerCells.Storage
import frc.robot.subsystems.Climber.ClimberSubsystem
import frc.robot.subsystems.ColorWheel.ActuatorSubsystem
import frc.robot.subsystems.Drivetrain.DriveSubsystem
import frc.robot.subsystems.Inputs.JoystickSubsystem
import frc.robot.subsystems.Inputs.Jetson.VisionSubsystem
import frc.robot.subsystems.PowerCells.IntakeSubsystem
import frc.robot.subsystems.PowerCells.ShooterSubsystem
import frc.robot.subsystems.PowerCells.TransportSubsystem
import java.util.List
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

    private val driverJoystick: Joystick
//    private val alternateJoystick: Joystick

    /** Subsystems*/
    private val m_driverJoystickSubsystem = JoystickSubsystem(Constants.driverJoystickPort)
//    private val m_alternateJoystickSubsystem = JoystickSubsystem(Constants.alternateJoystickPort)

//    private val m_xboxSubsystem = XboxSubsystem()


    private val m_driveSubsystem = DriveSubsystem()
    private val m_intakeSubsystem = IntakeSubsystem()
//    private val m_controlPanelSubsystem = SpinSubsystem()
    private val m_actuatorSubsystem = ActuatorSubsystem()
    private val m_climberSubsystem = ClimberSubsystem()
//    private val m_LidarSubsystem = LidarSubsystem()
    private val m_visionSubsystem = VisionSubsystem(Constants.visionHost, Constants.visionPort)
    private val m_shooterSubsystem = ShooterSubsystem()
    private val m_transportSubsystem = TransportSubsystem()

    /**Commands */
    private val m_defaultDrive = DefaultDrive(m_driveSubsystem, m_driverJoystickSubsystem)

    private val Lifter = Lifter(m_climberSubsystem)
    private val Winch = Winch(m_climberSubsystem)
//    private val Actuator = Actuator(m_controlPanelSubsystem )
    private val Intake = Intake(m_intakeSubsystem)
    private val shooter = Shooter(m_shooterSubsystem)
    private val storage = Storage(m_transportSubsystem)
    private val Bar = RotateIntakeBarTo(m_intakeSubsystem)


    var runTime = 0.2
    var waitTime = 0.5

    val shoot = storage.ForceRun(-0.5).andThen(WaitCommand(0.2)).andThen(storage.Stop()).andThen(WaitCommand(1.0))
            .andThen(storage.Run()).andThen(WaitCommand(runTime)).andThen(storage.Stop()).andThen(WaitCommand(waitTime))
            .andThen(storage.Run()).andThen(WaitCommand(runTime)).andThen(storage.Stop()).andThen(WaitCommand(waitTime))
            .andThen(storage.Run()).andThen(WaitCommand(runTime)).andThen(storage.Stop()).andThen(WaitCommand(waitTime))
            .andThen(storage.Run()).andThen(WaitCommand(runTime)).andThen(storage.Stop()).andThen(WaitCommand(waitTime))
            .andThen(storage.Run()).andThen(WaitCommand(runTime)).andThen(storage.Stop()).andThen(WaitCommand(waitTime))
            .andThen(storage.Run()).andThen(WaitCommand(runTime)).andThen(storage.Stop()).andThen(WaitCommand(waitTime))


    val config: TrajectoryConfig = TrajectoryConfig(Constants.forwardMaxVel,
            Constants.forwardMaxAcc) // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(m_driveSubsystem.kDriveKinematics)

    var exampleTrajectory: Trajectory = TrajectoryGenerator.generateTrajectory( // Start at the origin facing the +X direction
            Pose2d(0.0, 0.0, Rotation2d(0.0)),  // Pass through these two interior waypoints, making an 's' curve path
            List.of(
                    Translation2d(1.0, 1.0),
                    Translation2d(2.0, -1.0)
            ),  // End 3 meters straight ahead of where we started, facing forward
            Pose2d(3.0, 0.0, Rotation2d(0.0)),
            config
    )


    /**
     * Use this method to define your button->command mappings.  Buttons can be created by
     * instantiating a GenericHID or one of its subclasses, and then passing it to a
     * [edu.wpi.first.wpilibj2.command.button.JoystickButton].
     */
    private fun configureButtonBindings() {
        //Run Interior to Shoot WHILE shooter is revved
        var tempMaxVel = 0.0
        var tempMaxAcc = 0.0
        //Spool Shooter
        JoystickButton(driverJoystick, 2).whenPressed(shooter.ForceRun(-0.3).alongWith(storage.ForceRun(0.3))).whenReleased(shooter.Stop().alongWith(storage.Stop()))


        JoystickButton(driverJoystick, 1).whenPressed(shooter.ForceRun(0.65).alongWith(shoot)).whenInactive(shooter.Stop().alongWith(storage.Stop()))
        //Run Intake
        JoystickButton(driverJoystick, 3).or(JoystickButton(driverJoystick, 4))
                .whileActiveOnce(Bar.moveDown().andThen(Intake.Run()))
                .whenInactive(Intake.Stop().andThen(Bar.moveUp()))

        //Reverse Intake
         JoystickButton(driverJoystick, 5).or(JoystickButton(driverJoystick, 6))
                 .whileActiveOnce(Bar.moveDown().andThen(Intake.Reverse()))
                 .whenInactive(Intake.Stop().andThen(Bar.moveUp()))



        JoystickButton(driverJoystick, 3).whenPressed(Intake.Run()).whenReleased(Intake.Stop())

        JoystickButton(driverJoystick, 4).whenPressed(Bar)

        JoystickButton(driverJoystick, 12).whenPressed(Bar.moveUp())
        JoystickButton(driverJoystick, 10).whenPressed(Bar.moveDown())
        JoystickButton(driverJoystick, 6).whenPressed(Runnable { m_intakeSubsystem.m_intakeDeploy.selectedSensorPosition = 0})

        JoystickButton(driverJoystick, 5).whileHeld(Runnable {
            tempMaxVel = Math.max(tempMaxVel, m_driveSubsystem.maxSpeed)
            tempMaxAcc = Math.max(tempMaxAcc, m_driveSubsystem.getWheelAcc())
            println("maxSpeed: $tempMaxVel \nmaxAcc: $tempMaxAcc")
        })
        //Rotate Counter-Clockwise
//        JoystickButton(driverJoystick, 9).whenPressed(RotateTo(-90.0, m_driveSubsystem).withTimeout(1.5))
//
//        //Rotate Clockwise
//        JoystickButton(driverJoystick, 10).whenPressed(RotateTo(90.0, m_driveSubsystem).withTimeout(1.5))



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

        driverJoystick = m_driverJoystickSubsystem.joystick
//        alternateJoystick = m_alternateJoystickSubsystem.joystick
        m_visionSubsystem.startUp()


        // Configure the button bindings

        configureButtonBindings()
        m_defaultDrive.initialize()

    }

    fun getCartesianDrive():Command {
        return m_defaultDrive
    }


    fun generatePathfindingCommand(trajectory: Trajectory): Command {
        return MecanumControllerCommand(
                trajectory,
                Supplier { m_driveSubsystem.getMPose() },
                SimpleMotorFeedforward(Constants.ks, Constants.kv, Constants.ka),
                m_driveSubsystem.kDriveKinematics,
                PIDController(Constants.xP, Constants.xI, Constants.xD),
                PIDController(Constants.yP, Constants.yI, Constants.yD),
                ProfiledPIDController(Constants.tP, Constants.tI, Constants.tD, TrapezoidProfile.Constraints(Constants.maxRotVel, Constants.maxRotAcc)),//@TODO FIX THIS
                5.0,  //@TODO FIX THIS
                PIDController(0.00239, 0.0, 0.0),
                PIDController(0.00239, 0.0, 0.0),
                PIDController(0.00239, 0.0, 0.0),
                PIDController(0.00239, 0.0, 0.0),
                Supplier { m_driveSubsystem.getWheelSpeeds() },
                Consumer { output -> m_driveSubsystem.setSpeedVoltage(output)},
                m_driveSubsystem
        )
    }

    fun autoPathSimple():Command {
        return DriveToPowerPort(m_driveSubsystem, m_visionSubsystem, 0.3).andThen(
                shoot.withTimeout(4.0)
        ).andThen(
                ForceDrive(m_driveSubsystem, -0.75, 0.0, 0.0)
        ).andThen(
                WaitCommand(3.0)
        ).andThen(
                ForceDrive(m_driveSubsystem, 0.0, 0.0, 0.0)
        )
    }

}
