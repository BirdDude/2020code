/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Drivetrain;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain.DriveSubsystem;
import frc.robot.subsystems.Inputs.Jetson.VisionSubsystem;

/**
 * An example command that uses an example subsystem.
 */
public class RotateToPowerPort extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private DriveSubsystem driveSubsystem;
  private VisionSubsystem visionSubsystem;
  private Joystick joystick;
  private PIDController controller;

  public RotateToPowerPort(DriveSubsystem drive, VisionSubsystem vision, Joystick stick) {
    driveSubsystem = drive;
    visionSubsystem = vision;
    joystick = stick;
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
    driveSubsystem.driveCartesan(-joystick.getY(), joystick.getX(), -output / 51.43);
    System.out.println(visionSubsystem.getPowerPortBearing());
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
    return false;
  }


}
