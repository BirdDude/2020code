/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Drivetrain;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.MecanumControllerCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

import java.util.ArrayList;

/**
 * An example command that uses an example subsystem.
 */
public class PathfindTo extends MecanumControllerCommand {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveSubsystem m_driveSubsystem;

  /**
   * Creates a new ExampleCommand.
   *
   * param subsystem The subsystem used by this command.
   */
  public PathfindTo(double x, Double y, Double r, DriveSubsystem drive) {
    super(
            TrajectoryGenerator.generateTrajectory(new Pose2d(0.0, 0.0, new Rotation2d(0.0)), new ArrayList<>(), new Pose2d(x, y, new Rotation2d(r)),
                    new TrajectoryConfig(Constants.maxVel, Constants.maxAcc).setKinematics(drive.getKDriveKinematics()).addConstraint(drive.getKDriveConstraints())),
            drive::getM_Pose,
            drive.getKDriveKinematics(),
            new PIDController(Constants.xP, Constants.xI, Constants.xD, Constants.dt),
            new PIDController(Constants.yP, Constants.yI, Constants.yD, Constants.dt),
            new ProfiledPIDController(Constants.tP, Constants.tI, Constants.tD, new TrapezoidProfile.Constraints(Constants.maxRotVel, Constants.maxRotAcc), Constants.dt),
            Constants.maxVel,
            drive::setmManualWheelSpeeds,
            drive
    );


    m_driveSubsystem = drive;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveSubsystem);
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveSubsystem.driveCartesan(0.0, 0.0, 0.0);
  }

}
