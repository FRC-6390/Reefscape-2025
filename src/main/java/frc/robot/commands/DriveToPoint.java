// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import ca.frc6390.athena.core.RobotLocalization;
import ca.frc6390.athena.drivetrains.swerve.SwerveDrivetrain;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveToPoint extends Command {
  public RobotLocalization localization;
  public Pose2d goal;
  public ProfiledPIDController xController = new ProfiledPIDController(0, 0, 0, new Constraints(0, 0));
  public ProfiledPIDController yController = new ProfiledPIDController(0, 0, 0, new Constraints(0, 0));;
  public ProfiledPIDController rotationController = new ProfiledPIDController(0, 0, 0, new Constraints(0, 0));;
  public SwerveDrivetrain drivetrain;
  public boolean shouldResetPose;
  public Pose2d startPose;
  public boolean isDone;

  public DriveToPoint(RobotLocalization localization, SwerveDrivetrain drivetrain, Pose2d goal, Pose2d startPose, boolean shouldResetPose) {
    this.localization = localization;
    this.drivetrain = drivetrain;
    this.shouldResetPose = shouldResetPose;
    this.startPose = startPose;
    isDone = false;
  }

  @Override
  public void initialize() 
  {
    isDone = false;
    if(shouldResetPose)
    {
    localization.resetFieldPose(startPose);
    }
  }

  @Override
  public void execute() 
  {
    double xSpeed = xController.calculate(localization.getFieldPose().getX(), startPose.getX());
    double ySpeed = yController.calculate(localization.getFieldPose().getY(), startPose.getY());
    double thetaSpeed = rotationController.calculate(localization.getPose().getRotation().getDegrees(), startPose.getRotation().getDegrees());
    
    ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(ySpeed, xSpeed, thetaSpeed, drivetrain.getIMU().getVirtualAxis("auto"));

    drivetrain.getRobotSpeeds().setFeedbackSpeeds(chassisSpeeds);   
    if(xController.atGoal() &&  yController.atGoal() && rotationController.atGoal())
    {
      isDone = true;
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return isDone;
  }
}
