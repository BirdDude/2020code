/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain.DriveSubsystem;
import frc.robot.subsystems.Inputs.Jetson.VisionSubsystem;

/**
 * An example command that uses an example subsystem.
 */
public class DriveToPowerPort extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private DriveSubsystem driveSubsystem;
  private VisionSubsystem visionSubsystem;
  private PIDController controller;
  private Double distance;

  public DriveToPowerPort(DriveSubsystem drive, VisionSubsystem vision, Double targetDistance) {
    driveSubsystem = drive;
    visionSubsystem = vision;
    distance = targetDistance;
    controller = new PIDController(Constants.tP, Constants.tI, Constants.tD);
    controller.enableContinuousInput(-180, 180);
    controller.setTolerance(0.1, 0.2);
    addRequirements(drive, vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //   try {
    //     visionSubsystem.startUp();
    //   } catch (IOException e) {
    //     // TODO Auto-generated catch block
    //     e.printStackTrace();
    //   }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double output = controller.calculate(visionSubsystem.getPowerPortBearing(), 0.0);

    if (visionSubsystem.getPowerDistance() > 0.0) driveSubsystem.driveCartesan(visionSubsystem.getPowerDistance() / 5.0, 0.0, -output / 51.43);
    else driveSubsystem.driveCartesan(0.0, 0.0, -output / 51.43);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // try {
    //   visionSubsystem.shutDown();
    // } catch (IOException e) {
    //   // TODO Auto-generated catch block
    //   e.printStackTrace();
    // }
    driveSubsystem.driveCartesan(0.0, 0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return controller.atSetpoint() && visionSubsystem.getPowerDistance() <= distance;
  }


}
