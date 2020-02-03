/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.Logic.VisionComms;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/**
 * An example command that uses an example subsystem.
 */
public class RotateTo extends PIDCommand {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})


/**
 * Turns to robot to the specified angle.
 *
 * @param targetAngleDegrees The angle to turn to
 * @param drive              The drive subsystem to use
 */
  public RotateTo(double targetAngleDegrees, DriveSubsystem drive, VisionSubsystem vision) {
    super(
            new PIDController(Constants.tP, Constants.tI, Constants.tD),

            // Close loop on heading
            vision::getLoadingBearing,

            // Set reference to target
            targetAngleDegrees,
            // Pipe output to turn robot
            output -> drive.driveCartesanRobot(0.0, 0.0, output / 180.0),
            // Require the drive
            drive);


    // Set the controller to be continuous (because it is an angle controller)
    getController().enableContinuousInput(-180, 180);
    // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
    // setpoint before it is considered as having reached the reference
    getController().setTolerance(5.0, 10.0);

  }

  @Override
  public boolean isFinished() {
    // End when the controller is at the reference.
    return getController().atSetpoint();
  }

}