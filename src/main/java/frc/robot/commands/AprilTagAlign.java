// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import ca.frc6390.athena.controllers.DebouncedController;
import ca.frc6390.athena.drivetrains.swerve.SwerveDrivetrain;
import ca.frc6390.athena.sensors.camera.limelight.LimeLight;
import ca.frc6390.athena.sensors.camera.limelight.LimelightConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class AprilTagAlign extends Command {
  public LimeLight limelight; 
  public SwerveDrivetrain drivetrain;
  public DebouncedController cont; 
  // public PIDController controller = new PIDController(0.025, 0, 0);
  // public PIDController xController = new PIDController(1.225, 0, 0);
  public PIDController controller = new PIDController(0.025, 0, 0);
  public PIDController xController = new PIDController(1, 0, 0);
  public double thetaSpeed = 0;

  public AprilTagAlign(String limeLight, SwerveDrivetrain drivetrain, DebouncedController cont) {
    this.drivetrain = drivetrain; 
    this.limelight = new LimeLight(new LimelightConfig(limeLight)); 
    this.cont = cont;
  }

  @Override
  public void initialize() 
  {
  }

  @Override
  public void execute() 
  {
   
    if(limelight.hasValidTarget()){
        if(Math.abs(limelight.getTargetHorizontalOffset()) > 15 && limelight.getTargetArea() < 1000)
        {
         Pose2d poseEstimate = limelight.getPoseEstimate(LimeLight.PoseEstimateType.BOT_POSE_TARGET_SPACE).getPose();
         double rotationalVelocity = controller.calculate(poseEstimate.getRotation().getDegrees());
         ChassisSpeeds speeds = new ChassisSpeeds(cont.leftY.getAsDouble(), 0, rotationalVelocity); 
         ChassisSpeeds fieldRelative = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, poseEstimate.getRotation());
         drivetrain.drive(fieldRelative);

        }
        else
        {
         Pose2d poseEstimate = limelight.getPoseEstimate(LimeLight.PoseEstimateType.BOT_POSE_TARGET_SPACE).getPose();
         double rotationalVelocity = controller.calculate(poseEstimate.getRotation().getDegrees());
         double xVelocity = -xController.calculate(limelight.getTargetHorizontalOffset());
         ChassisSpeeds speeds = new ChassisSpeeds(cont.leftY.getAsDouble(), xVelocity, rotationalVelocity); 
         ChassisSpeeds fieldRelative = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, poseEstimate.getRotation());
         drivetrain.drive(fieldRelative);
        }}






    // if(DriverStation.isTeleop())
    // {
    // if(limelight.hasValidTarget()){
    //   if(Math.abs(limelight.getTargetHorizontalOffset()) > 35)
    //   {
    //     drivetrain.addFeedbackSpeed(
    //     new ChassisSpeeds(
    //       cont.leftY.getAsDouble(), 
    //       0, 
    //       // controller.calculate(LimelightHelpers.getBotPose_TargetSpace(limelight)[4])
    //       controller.calculate(limelight.getPoseEstimate(LimeLight.PoseEstimateType.BOT_POSE_TARGET_SPACE).getPose().getRotation().getDegrees()))
    //       );
    //   }
    //   else
    //   {
    //     drivetrain.addFeedbackSpeed(
    //       new ChassisSpeeds(
    //         cont.leftY.getAsDouble(), 
    //         -xController.calculate(limelight.getTargetHorizontalOffset()), 
    //         // controller.calculate(LimelightHelpers.getBotPose_TargetSpace(limelight)[4])
    //         controller.calculate(limelight.getPoseEstimate(LimeLight.PoseEstimateType.BOT_POSE_TARGET_SPACE).getPose().getRotation().getDegrees()))
    //         );
    //   }

    //    }
    //   }
    // else if(DriverStation.isAutonomous())
    // {
    // if(limelight.hasValidTarget()){
    //   if(Math.abs(limelight.getTargetHorizontalOffset()) > 35)
    //   {
    //     drivetrain.drive(new ChassisSpeeds(drivetrain.getDriveSpeeds().vxMetersPerSecond, 0, drivetrain.getDriveSpeeds().omegaRadiansPerSecond));
    //     drivetrain.addFeedbackSpeed(
    //     new ChassisSpeeds(
    //       0, 
    //       0, 
    //       // controller.calculate(LimelightHelpers.getBotPose_TargetSpace(limelight)[4])
    //       controller.calculate(limelight.getPoseEstimate(LimeLight.PoseEstimateType.BOT_POSE_TARGET_SPACE).getPose().getRotation().getDegrees()))
    //       );
    //   }
    //   else
    //   {
    //     drivetrain.addFeedbackSpeed(
    //       new ChassisSpeeds(
    //         cont.leftY.getAsDouble(), 
    //         -xController.calculate(limelight.getTargetHorizontalOffset()), 
    //         // controller.calculate(LimelightHelpers.getBotPose_TargetSpace(limelight)[4])
    //         controller.calculate(limelight.getPoseEstimate(LimeLight.PoseEstimateType.BOT_POSE_TARGET_SPACE).getPose().getRotation().getDegrees()))
    //         );
    //   }

    //    }
    //    else
    //    {
    //     drivetrain.drive(drivetrain.getDriveSpeeds());
    //     drivetrain.addFeedbackSpeed(new ChassisSpeeds(0,0,0));
    //    }
    //   }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    drivetrain.drive(new ChassisSpeeds(0,0,0));
    drivetrain.addFeedbackSpeed(new ChassisSpeeds(0,0,0));
    drivetrain.drive(drivetrain.getDriveSpeeds());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
