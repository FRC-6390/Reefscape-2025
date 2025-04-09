

package frc.robot.commands.auto;
 
import ca.frc6390.athena.controllers.DelayedOutput;
 import ca.frc6390.athena.core.RobotBase;
import ca.frc6390.athena.core.RobotSpeeds;
import ca.frc6390.athena.sensors.camera.limelight.LimeLight;
 import ca.frc6390.athena.sensors.camera.limelight.LimeLight.PoseEstimateType;
 import ca.frc6390.athena.sensors.camera.limelight.LimeLight.PoseEstimateWithLatencyType;

import java.util.List;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
 import edu.wpi.first.math.controller.ProfiledPIDController;
 import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryGenerator.ControlVectorList;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
 import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
 import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utils.ReefScoringPos.ReefPole;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
 
 public class V2 extends Command {

  public LimeLight ll;
  public RobotSpeeds robotSpeeds;
  public RobotBase<?> base;
  public Trajectory trajectory; 
  public double startTime;
  public HolonomicDriveController controller = new HolonomicDriveController(
                                                          new PIDController(0, 0, 0), 
                                                          new PIDController(0, 0, 0),
                                                          new ProfiledPIDController(0, 0, 0, new Constraints(0, 0)));



   public V2(RobotBase<?> base,String lltable)
   {
    ll = base.getVision().getLimelight(lltable);
    this.base = base;
    robotSpeeds = base.getRobotSpeeds();
    List<Pose2d> list = List.of(new Pose2d());
    TrajectoryGenerator.generateTrajectory(list, new TrajectoryConfig(null, null));

   }

   @Override
   public void initialize() 
   {
    startTime = System.currentTimeMillis();
    if(ll.hasValidTarget())
    {
      base.getLocalization()
      .resetFieldPose
      (
        getBotPoseTagSpace(ll)
        .transformBy(new Transform2d(new Translation2d(), ReefPole.getPoleFromID(ll.getAprilTagID(), ll).getPose2d().getRotation()))
      );
    } 

    trajectory = TrajectoryGenerator.generateTrajectory(List.of(base.getLocalization().getFieldPose(), new Pose2d()), new TrajectoryConfig(1, 1));
   }
 

  
   public Pose2d getBotPoseTagSpace(LimeLight ll)
   {
     double dist = ll.getPoseEstimate(PoseEstimateWithLatencyType.BOT_POSE_MT2_BLUE).getRaw()[9];
     double angle = ll.getTargetHorizontalOffset();
     double x = (Math.cos(Math.toRadians(angle)) * dist);
     double y = (Math.sin(Math.toRadians(angle)) * dist); 
 
     return new Pose2d(x,y,base.getLocalization().getFieldPose().getRotation());
   }
 
   // Called every time the scheduler runs while the command is scheduled.
   @Override
   public void execute() 
   {
    if(ll.hasValidTarget())
    {
      base.getLocalization()
      .resetFieldPose
      (
        getBotPoseTagSpace(ll)
        .transformBy(new Transform2d(new Translation2d(), ReefPole.getPoleFromID(ll.getAprilTagID(), ll).getPose2d().getRotation()))
      );
    } 

    Trajectory.State goal = trajectory.sample(Units.millisecondsToSeconds(startTime - System.currentTimeMillis()));
    base.getDrivetrain().getRobotSpeeds().setSpeeds("feedback", controller.calculate(base.getLocalization().getFieldPose(), goal, goal.poseMeters.getRotation()));
   }
 
 
   // Called once the command ends or is interrupted.
   @Override
   public void end(boolean interrupted) {
     
     

   }
 
   // Returns true when the command should end.
   @Override
   public boolean isFinished() {
     
     return false;
     
   }
 }