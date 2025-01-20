// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.signals.RobotEnableValue;

import ca.frc6390.athena.controllers.DebouncedController;
import ca.frc6390.athena.core.RobotLocalization;
import ca.frc6390.athena.core.RobotVision;
import ca.frc6390.athena.drivetrains.swerve.SwerveDrivetrain;
import ca.frc6390.athena.sensors.camera.limelight.LimeLight;
import ca.frc6390.athena.sensors.camera.limelight.LimelightConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class AprilTagAlign extends Command {
  public LimeLight limelight; 
  public SwerveDrivetrain drivetrain;
  public DebouncedController cont; 
  // public PIDController controller = new PIDController(0.025, 0, 0);
  // public PIDController xController = new PIDController(1.225, 0, 0);
  public PIDController controller = new PIDController(0.05, 0, 0);
  public PIDController xController = new PIDController(0.05, 0, 0);
  public double thetaSpeed = 0;
  public Pose2d targetRed = new Pose2d(13.043, 4.007, new Rotation2d());
  public RobotVision vision;
  public RobotLocalization localization;
  public NetworkTable table;
  public NetworkTableEntry raw;

  public AprilTagAlign(RobotVision vision, RobotLocalization localization, SwerveDrivetrain drivetrain, DebouncedController cont) {
    this.drivetrain = drivetrain; 
    this.vision = vision;
    this.cont = cont;
    this.localization = localization;
    limelight = vision.getCamera("limelight-driver");
  }

  @Override
  public void initialize() 
  {
    // controller.enableContinuousInput(-Math.PI, Math.PI);
    controller.setTolerance(0.3);
    table = NetworkTableInstance.getDefault().getTable("limelight-driver");
    raw = table.getEntry("rawfiducials");
    
  }


  @Override
  public void execute() 
  {
    
    
    if(limelight.hasValidTarget()){
      SmartDashboard.putNumber("X", limelight.getTargetHorizontalOffset());
      SmartDashboard.putNumber("Raw X", (double)limelight.getRawFiducials()[1]);
        // if(Math.abs(limelight.getTargetHorizontalOffset()) > 25)
        // {
        // double xdiff = targetRed.getX() - localization.getPose().getX();
        // double ydiff = targetRed.getY() - localization.getPose().getY();
        // double angle = Math.atan2(ydiff, xdiff);
        double rotationalVelocity = controller.calculate(limelight.getTargetHorizontalOffset());

        double xVelocity = -xController.calculate(limelight.getRawFiducials()[1].doubleValue());
        ChassisSpeeds speeds = new ChassisSpeeds(cont.leftY.getAsDouble(), xVelocity, rotationalVelocity); 
        //  ChassisSpeeds fieldRelative = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, poseEstimate.getRotation());
         drivetrain.drive(speeds);
        // }
        // else
        // {
        //  Pose2d poseEstimate = limelight.getPoseEstimate(LimeLight.PoseEstimateType.BOT_POSE_TARGET_SPACE).getPose();
        //  double rotationalVelocity = controller.calculate(poseEstimate.getRotation().getDegrees());
        //  double xVelocity = -xController.calculate(limelight.getTargetHorizontalOffset());
        //  ChassisSpeeds speeds = new ChassisSpeeds(cont.leftY.getAsDouble(), xVelocity, rotationalVelocity); 
        //  ChassisSpeeds fieldRelative = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, poseEstimate.getRotation());
        //  drivetrain.drive(fieldRelative);
        // }
      }
      else{
        drivetrain.drive(new ChassisSpeeds(0,0,0));

      }


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
    SmartDashboard.putNumber("Is Running", 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
