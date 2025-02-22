// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import au.grapplerobotics.LaserCan;
import ca.frc6390.athena.controllers.EnhancedXboxController;
import ca.frc6390.athena.core.RobotBase;
import ca.frc6390.athena.core.RobotLocalization;
import ca.frc6390.athena.drivetrains.swerve.SwerveDrivetrain;
import ca.frc6390.athena.filters.FilterList;
import ca.frc6390.athena.filters.FilteredValue;
import ca.frc6390.athena.sensors.camera.limelight.LimeLight;
import ca.frc6390.athena.sensors.camera.limelight.LimeLight.PoseEstimateType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utils.AutoAlignHelper;
import frc.robot.utils.ReefScoringPos;
import frc.robot.utils.ReefScoringPos.ReefPole;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PassiveAlign extends Command {

  public LimeLight limeLight;
  public RobotLocalization localization;
  public AutoAlignHelper helper;
  public LaserCan las;
  public RobotBase<?> base;
  public ProfiledPIDController rController = new ProfiledPIDController(1.2, 0, 0.0, new Constraints(1, 1));
  public ProfiledPIDController xController = new ProfiledPIDController(0.06, 0, 0.0, new Constraints(50, 75));
  public FilteredValue rotationFiltered;
  public FilteredValue xOffsetFiltered;
  
  /** Creates a new PassiveAlign. */
  public PassiveAlign(RobotBase<?> base, LaserCan las) {
    this.base = base;
    this.localization = base.getLocalization();
    this.las = las;
    rController.enableContinuousInput(-Math.PI, Math.PI);
    rotationFiltered =  new FilteredValue(() -> limeLight.getPoseEstimate(PoseEstimateType.TARGET_POSE_ROBOT_SPACE).getRaw()[4])
                        .addMovingAverage(10)
                        .addMedianFilter(15);
    xOffsetFiltered =  new FilteredValue(() -> limeLight.getTargetHorizontalOffset())
                        .addMovingAverage(5);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    limeLight = base.getCameraFacing(ReefPole.A.getTranslation());
    helper = new AutoAlignHelper(limeLight, localization, base.getDrivetrain());
    xController.reset(0);
    rController.reset(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    if(limeLight.hasValidTarget())
    {

      Pose2d fieldPose = localization.getFieldPose();
      long id = limeLight.getAprilTagID();

      ReefPole pole = ReefPole.getPoleFromID(id);

      Rotation2d targetPose = pole.getRotation().plus(limeLight.config.getRotationRelativeToForwards());
     
      
      double r = rController.calculate(fieldPose.getRotation().getRadians(), MathUtil.angleModulus(targetPose.getRadians()));   
      double x = limeLight.config.getAngleCos() * xController.calculate(xOffsetFiltered.getFiltered(),0);
      base.getDrivetrain().getRobotSpeeds().setFeedbackSpeeds(0,x,r);
      
      SmartDashboard.putString("target", pole.name());
      SmartDashboard.putNumber("target angle", targetPose.getDegrees());
      SmartDashboard.putNumber("current angle", fieldPose.getRotation().getDegrees());

      SmartDashboard.putNumber("xOffsetFiltered", xOffsetFiltered.get());

      // if(limeLight.getTargetHorizontalOffset() < 50 && las.getMeasurement().status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT)
      // {
      //   if(las.getMeasurement().distance_mm <  2000)
      //   {
      //   base.getDrivetrain().getRobotSpeeds().setDriverSpeeds(0,x,r);
      //   }
      // }
    }
    else{
      base.getDrivetrain().getRobotSpeeds().setFeedbackSpeeds(0,0,0);
      
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    base.getDrivetrain().getRobotSpeeds().stopFeedbackSpeeds();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
