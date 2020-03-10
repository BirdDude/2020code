/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain.DriveSubsystem;
import frc.robot.subsystems.Inputs.Jetson.VisionSubsystem;
import frc.robot.subsystems.PowerCells.ShooterSubsystem;
import frc.robot.subsystems.PowerCells.TransportSubsystem;

public class Auto extends CommandBase {
  private DriveSubsystem driveSubsystem;
  private VisionSubsystem visionSubsystem;

  private Double distanceToPort = 2.0;
  private boolean atPort = false;
  private PIDController rotationController = new PIDController(Constants.tP, Constants.tI, Constants.tD);
  private PIDController xController = new PIDController(1.0, 0.0, 0.0);
  private PIDController yController = new PIDController(1.0, 0.0, 0.0);
  private double timeOut = 10.0;

  public Auto(DriveSubsystem drive, VisionSubsystem vision) {
    driveSubsystem = drive;
    visionSubsystem = vision;
    addRequirements(drive, vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (visionSubsystem.getPowerPortBearing() != Double.MIN_VALUE) {
      driveSubsystem.autoCartesian(0.0, 0.0, 0.75);
    } else {
      if (visionSubsystem.getPowerDistance() > distanceToPort) {
        dirveToPowerPort();
      } else {
        atPort = true;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.autoCartesian(0.0, 0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return atPort;
  }

  private void dirveToPowerPort() {
    double xOutput = xController.calculate(visionSubsystem.getPowerPortBearing(), 0.0) / 3.0;
    double yOutput = yController.calculate(visionSubsystem.getLoadingBayDistanc(), distanceToPort) / 3.0;
    double rotationOutput = rotationController.calculate(driveSubsystem.getHeading(), 0.0) / 180.0;

    driveSubsystem.autoCartesian(xOutput, yOutput, rotationOutput);
  }
}
