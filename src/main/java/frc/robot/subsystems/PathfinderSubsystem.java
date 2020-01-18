/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.MecanumDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.constraint.MecanumDriveKinematicsConstraint;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.trajectory.*;
import frc.robot.Constants;

public class PathfinderSubsystem extends SubsystemBase {

  Translation2d frontRight = new Translation2d(0.381, 0.381);
  Translation2d frontLeft = new Translation2d(-0.381, 0.381);
  Translation2d backRight = new Translation2d(0.381, -0.381);
  Translation2d backLeft = new Translation2d(-0.381, -0.381);



  MecanumDriveKinematics kDriveKinematics = new MecanumDriveKinematics(frontLeft, frontRight, backLeft, backRight);
  MecanumDriveKinematicsConstraint kDriveMecanum = new MecanumDriveKinematicsConstraint(kDriveKinematics, Constants.INSTANCE.getMaxVel());

  public PathfinderSubsystem() {


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
