// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.auto;

// import ca.frc6390.athena.controllers.DelayedOutput;
// import ca.frc6390.athena.core.RobotBase;
// import ca.frc6390.athena.core.RobotSpeeds.SpeedSource;
// import ca.frc6390.athena.sensors.camera.limelight.LimeLight;
// import ca.frc6390.athena.sensors.camera.limelight.LimeLight.PoseEstimateType;
// import ca.frc6390.athena.sensors.camera.limelight.LimeLight.PoseEstimateWithLatencyType;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.filter.MedianFilter;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.CommandScheduler;
// import edu.wpi.first.wpilibj2.command.Commands;
// import frc.robot.subsystems.superstructure.CANdleSubsystem;
// import frc.robot.utils.ReefScoringPos.ReefPole;

// public class TagAlign extends Command {
//   public RobotBase<?> base;
//   public double AutoDistToTrigger;
//   public static double horizonalTolerance = 2.5;
//   public double DistToCommand;
//   public Command command;
//   public static LimeLight ll;
//   public int runTag;
//   public Pose2d curPose;
//   public MedianFilter filter;
//   public double thetaMeasurement;
//   public ProfiledPIDController xController = new ProfiledPIDController(2, 0, 0, new Constraints(0.25, 0.25));
//   public ProfiledPIDController yController =  new ProfiledPIDController(4, 0.1, 0, new Constraints(1, 1));
//   public PIDController rController = new PIDController(0.04, 0, 0);
//   public DelayedOutput endCommand;
//   public DelayedOutput noTag;
//   public ReefPole pole;
//   public CANdleSubsystem candle;
  

//   public TagAlign(RobotBase<?> base,String lltable, Command command, double DistToCommand, ReefPole pole, CANdleSubsystem candle)
//   {
//    this.base = base;
//    this.pole = pole;
//    this.command = command;
//    this.candle = candle;
//    this.DistToCommand = DistToCommand;
//    ll = this.base.getVision().getLimelight(lltable);
//   }

//   public TagAlign(RobotBase<?> base, String lltable, CANdleSubsystem candle)
//   {
//    this(base, lltable, Commands.none(), Double.POSITIVE_INFINITY, ReefPole.NONE, candle);
//   }

//   public TagAlign(RobotBase<?> base, String lltable, Command command, double DistToCommand, CANdleSubsystem caNdleSubsystem)
//   {
//    this(base, lltable, command, DistToCommand, ReefPole.NONE, caNdleSubsystem);
//   }


//   @Override
//   public void initialize() 
//   {
//     runTag = -1;
//     endCommand = new DelayedOutput(() -> closeEnough(), 0.75);
    
//     curPose = new Pose2d();
//     filter = new MedianFilter(25);
//     noTag = new DelayedOutput(() -> hasNoTag(), 1);
//     thetaMeasurement = 0;
//     rController.setIntegratorRange(-5, 5);
//     ll.setPriorityID((int)pole.getApriltagId());
//     xController.setIntegratorRange(-0.05, 0.05);
//     xController.reset(curPose.getX());
//     yController.reset(curPose.getY());

//     // yController.setIntegratorRange(-0.2, 0.2);
//   }

//   public boolean hasNoTag()
//   {
//     return !(runTag == (int)ll.getAprilTagID()) || !ll.hasValidTarget();
//   }
//   public Pose2d getBotPoseTagSpace(LimeLight ll)
//   {
//     double dist = ll.getPoseEstimate(PoseEstimateWithLatencyType.BOT_POSE_MT2_BLUE).getRaw()[9];
//     double offset = 0;
//     if(!pole.equals(ReefPole.NONE))
//     {
//       offset = pole.getOffsetInDegrees();
//     }

//     double angle = ll.getTargetHorizontalOffset();
//     double x = (Math.cos(Math.toRadians(angle)) * dist);
//     double y = (Math.sin(Math.toRadians(angle)) * dist); 
//     SmartDashboard.putNumber("x",x);
//     SmartDashboard.putNumber("Y", y);
//     SmartDashboard.putNumber("Angle", angle);
    

//     return new Pose2d(x,y, base.getLocalization().getFieldPose().getRotation());
//   }

//   @Override
//   public void execute() 
//   {
//     candle.setRGB(255, 0, 0);
//     SmartDashboard.putBoolean("ENd ",endCommand.getAsBoolean());
//     SmartDashboard.putNumber("Dist", ll.getPoseEstimate(PoseEstimateWithLatencyType.BOT_POSE_MT2_BLUE).getRaw()[9]);
//     if(ll.hasValidTarget())
//     { 
//       if(runTag == -1)
//       {
//         runTag = (int)ll.getAprilTagID();
//       }
//       curPose = getBotPoseTagSpace(ll);
//       thetaMeasurement =-filter.calculate(ll.getPoseEstimate(PoseEstimateType.TARGET_POSE_ROBOT_SPACE).getRaw()[4]);
//     }
//     else
//     {
//       thetaMeasurement = 0;
//     }

//     double Xspeed =0; //-xController.calculate(curPose.getX(), Units.inchesToMeters(18));
//     double YSpeed =0;// yController.calculate(curPose.getY(),0);
//     double rSpeed =0;// rController.calculate(thetaMeasurement, 0); 


  
//     if(pole.getApriltagId() != -1)
//     {

//     if(runTag == pole.getApriltagId())
//     {
//       xController.calculate(curPose.getX(), Units.inchesToMeters(18));
//       yController.calculate(curPose.getY(),0);
//       rController.calculate(thetaMeasurement, 0);
      
//      Xspeed =  -xController.getSetpoint().velocity;
//      YSpeed =  yController.getSetpoint().velocity;
//      rSpeed =  rController.getSetpoint();
//     }
//     else
//     {
//       Xspeed = 0;
//       YSpeed = 0;
//       rSpeed = 0;
//     }
//   }

//   // if(ll.getPoseEstimate(PoseEstimateWithLatencyType.BOT_POSE_MT2_BLUE).getRaw()[9] > 1)
//   // {
//   //   yController.setP(2);
//   // }
//   // else
//   // {
//   //   yController.setP(1);
//   // }

  
//     if(ll.hasValidTarget())
//     {
//       if(runTag == (int)ll.getAprilTagID())
//       {
  
//         if(ll.getPoseEstimate(PoseEstimateWithLatencyType.BOT_POSE_MT2_BLUE).getRaw()[9] <= DistToCommand)
//         {
//           // CommandScheduler.getInstance().schedule(command);
//         }
//         SmartDashboard.putNumber("XSpeed", Xspeed);
//         SmartDashboard.putNumber("YSpeed", YSpeed);

//         SmartDashboard.putNumber("RSpeed", rSpeed);

//         SmartDashboard.putBoolean("ALGIN TAKEN",true);
//         base.getDrivetrain().getRobotSpeeds().setSpeedSourceState("auto", false);
//         candle.setRGB(0, 255, 0);
//         base.getDrivetrain().getRobotSpeeds().setSpeeds("feedback", Xspeed, YSpeed, -rSpeed);
//       }
//       else
//       {
//         candle.setRGB(255, 0, 0);

//         SmartDashboard.putBoolean("ALGIN TAKEN",false);
//         base.getDrivetrain().getRobotSpeeds().setSpeedSourceState("auto", false);
//         base.getDrivetrain().getRobotSpeeds().stopSpeeds("feedback");
//       }
//   }
//     else
//     {
//       candle.setRGB(255, 0, 0);

//       SmartDashboard.putBoolean("ALGIN TAKEN",false);
//       base.getDrivetrain().getRobotSpeeds().setSpeedSourceState("auto", false);
//       base.getDrivetrain().getRobotSpeeds().setSpeeds("feedback", -0.2, YSpeed, rSpeed);
      
//     }
//   }

//   public boolean closeEnough()
//   {
//     double offset = 0;
//     if(!pole.equals(ReefPole.NONE))
//     {
//       offset = pole.getOffsetInDegrees();
//     }
//     return ll.hasValidTarget() && ll.getPoseEstimate(PoseEstimateWithLatencyType.BOT_POSE_MT2_BLUE).getRaw()[9] <= 0.5 && Math.abs(Math.abs(ll.getTargetHorizontalOffset()) - Math.abs(offset)) < horizonalTolerance;
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     candle.setRGB(0, 0, 0);

//     if(ll.hasValidTarget())
//     {
//     base.getLocalization().resetFieldPose(ll.getPoseEstimate(PoseEstimateWithLatencyType.BOT_POSE_BLUE).getLocalizationPose().getX(), ll.getPoseEstimate(PoseEstimateWithLatencyType.BOT_POSE_BLUE).getLocalizationPose().getY());
//     }
//     base.getDrivetrain().getRobotSpeeds().setSpeedSourceState("auto", true);
    
//     base.getDrivetrain().getRobotSpeeds().stopSpeeds("feedback");
//     base.getDrivetrain().getRobotSpeeds().stopSpeeds("auto");

//   }

//   @Override
//   public boolean isFinished() {
    
//     if(DriverStation.isAutonomous())
//     {
//       return endCommand.getAsBoolean() || noTag.getAsBoolean();
//     }
//     else
//     {
//     return noTag.getAsBoolean();
//     }
    
//   }
// }

package frc.robot.commands.auto;
 
import java.util.function.Supplier;

import ca.frc6390.athena.controllers.DelayedOutput;
 import ca.frc6390.athena.core.RobotBase;
import ca.frc6390.athena.core.RobotSpeeds;
import ca.frc6390.athena.sensors.camera.limelight.LimeLight;
 import ca.frc6390.athena.sensors.camera.limelight.LimeLight.PoseEstimateType;
 import ca.frc6390.athena.sensors.camera.limelight.LimeLight.PoseEstimateWithLatencyType;
 import edu.wpi.first.math.controller.PIDController;
 import edu.wpi.first.math.controller.ProfiledPIDController;
 import edu.wpi.first.math.filter.MedianFilter;
 import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
 import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
 import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.SuperstructureState;
import frc.robot.utils.ReefScoringPos.ReefPole;
 
 public class TagAlign extends Command {
   public RobotSpeeds robotSpeeds;
   public LimeLight ll;
   public long runTag;
   public Translation2d curTranslation;
   public MedianFilter filter;
   public double thetaMeasurement;
   public ProfiledPIDController xController = new ProfiledPIDController(1, 0, 0, new Constraints(1, 1));
   public ProfiledPIDController yController =  new ProfiledPIDController(3.85, 0, 0, new Constraints(2, 1.5));
   public PIDController rController = new PIDController(0.025, 0, 0);
   public DelayedOutput endCommand;
   public Superstructure superstructure;
   public DelayedOutput noTag;
   public Supplier<SuperstructureState> state;

   public TagAlign(RobotBase<?> base,String lltable, Superstructure superstructure, Supplier<SuperstructureState> state)
   {
    this.superstructure = superstructure;
    this.state = state;
    this.robotSpeeds = base.getDrivetrain().getRobotSpeeds();
    this.ll = base.getVision().getLimelight(lltable);
   }


   @Override
   public void initialize() 
   {
     runTag = -1;
     endCommand = new DelayedOutput(() -> closeEnough(), Units.millisecondsToSeconds(40));
     curTranslation = new Translation2d();
     filter = new MedianFilter(50);
     noTag = new DelayedOutput(() -> hasNoTag(), 0.6);
     thetaMeasurement = 0;
   }
 
   public boolean hasNoTag()
   {
     return !ll.hasValidTarget();
   }

  
   public Translation2d getBotPoseTagSpace(LimeLight ll)
   {
     double dist = ll.getPoseEstimate(PoseEstimateWithLatencyType.BOT_POSE_MT2_BLUE).getRaw()[9];
     double angle = getOffsetToTarget();
     double x = (Math.cos(Math.toRadians(angle)) * dist);
     double y = (Math.sin(Math.toRadians(angle)) * dist); 
 
     return new Translation2d(x,y);
   }
 
   // Called every time the scheduler runs while the command is scheduled.
   @Override
   public void execute() 
   {
        
     SmartDashboard.putBoolean("ENd ",endCommand.getAsBoolean());

    //  return ll.hasValidTarget() && ll.getPoseEstimate(PoseEstimateWithLatencyType.BOT_POSE_MT2_BLUE).getRaw()[9] <= 0.525 && Math.abs(ll.getTargetHorizontalOffset())< 3.5;

     System.out.println("RUNNNING");
     if(ll.hasValidTarget())
     { 
       if(runTag == -1)
       {
         runTag = ll.getAprilTagID();
       }
       curTranslation = getBotPoseTagSpace(ll);
       thetaMeasurement =-filter.calculate(ll.getPoseEstimate(PoseEstimateType.TARGET_POSE_ROBOT_SPACE).getRaw()[4]);
     }
     else
     {
       thetaMeasurement = 0;
     }
 
     if(ll.getPoseEstimate(PoseEstimateWithLatencyType.BOT_POSE_MT2_BLUE).getRaw()[9] > 1)
     {
       xController.setP(1);
     }
     else
     {
      if(DriverStation.isTeleop() || DriverStation.isTeleopEnabled())
      {
       xController.setP(0.5);
      }
      else if(DriverStation.isAutonomous() || DriverStation.isAutonomousEnabled())
      {
        xController.setP(0.5);
      }
     }
 
    
     SmartDashboard.putBoolean("at goal state",superstructure.getStateMachine().atGoalState());
     SmartDashboard.putBoolean("at state",superstructure.getStateMachine().isGoalState(SuperstructureState.L3));

     if(!ll.hasValidTarget())
     {
        SmartDashboard.putBoolean("ALGIN TAKEN",false);
        // robotSpeeds.stopSpeeds("feedback");
        robotSpeeds.setSpeeds("feedback",-0.25, 0, 0);
        return;
     }

     double offset = 0;
     if(DriverStation.isTeleop() || DriverStation.isTeleopEnabled())
     {
      offset = 12.5;
     }
     else if(DriverStation.isAutonomous() || DriverStation.isAutonomousEnabled())
     {
      offset = 12.5;
     }
       double Xspeed = -xController.calculate(curTranslation.getX(), Units.inchesToMeters(offset));
       double YSpeed = yController.calculate(curTranslation.getY(),0);
       double rSpeed = rController.calculate(thetaMeasurement, 0);//* Math.copySign(1, ll.getTargetHorizontalOffset()), 0);

       if(ll.getPoseEstimate(PoseEstimateWithLatencyType.BOT_POSE_MT2_BLUE).getRaw()[9] <= 100000000)
       {
         SmartDashboard.putBoolean("ALGIN TAKEN",true);
         robotSpeeds.setSpeeds("feedback",Xspeed, YSpeed, -rSpeed);
       }
      //  else if(noTag.getAsBoolean() && DriverStation.isTeleop()){
      //   robotSpeeds.setSpeeds("feedback",0, -0.1, 0);
      //  }
       else
       {
         SmartDashboard.putBoolean("ALGIN TAKEN",false);
         robotSpeeds.stopSpeeds("feedback");
       }


    if(DriverStation.isTeleop())
    {

      
    if(!endCommand.getAsBoolean() && ll.getPoseEstimate(PoseEstimateWithLatencyType.BOT_POSE_MT2_BLUE).getRaw()[9] <= 1 && !superstructure.getStateMachine().atState(state.get()))
    {
     if(state.get().equals(SuperstructureState.L1) ||state.get().equals(SuperstructureState.L2) ||state.get().equals(SuperstructureState.L3))
     {
      superstructure.setSuper(state.get());
     }
    //  else if(state.get().equals(SuperstructureState.L4))
    //  {
    //   superstructure.setSuper(SuperstructureState.L3);
    //  }
    }

     if(endCommand.getAsBoolean())
     {
      // if(state.get().equals(SuperstructureState.L4))
      // {
      //   superstructure.setSuper(SuperstructureState.L4);
      // }
      if(superstructure.hasPiece() && !superstructure.stateMachine.getGoalState().equals(SuperstructureState.Score))
      {
      
      if(superstructure.l4Ready() || superstructure.l3Ready() || superstructure.l2Ready() || superstructure.l1Ready())
      {
        if(!superstructure.drop()) {superstructure.setSuper(SuperstructureState.Score);}
      }
      else
      {
        if(!superstructure.drop()) {superstructure.setSuper(state.get());}
      }

      if(superstructure.drop())
      {
        superstructure.setSuper(SuperstructureState.HomePID);
      }
      }
     }
    
  }
    
   }
 
   public boolean closeEnough()
   {
     return ll.hasValidTarget() && ll.getPoseEstimate(PoseEstimateWithLatencyType.BOT_POSE_MT2_BLUE).getRaw()[9] <= 0.525 && Math.abs(getOffsetToTarget()) < 5;
   }

   public double getOffsetToTarget(){
      return ll.getTargetHorizontalOffset() + ReefPole.getPoleFromID(runTag, ll).getOffsetInDegrees();
   }
 
   // Called once the command ends or is interrupted.
   @Override
   public void end(boolean interrupted) {
     
     
    robotSpeeds.stopSpeeds("feedback");
    System.out.println("ENDED");

   }
 
   // Returns true when the command should end.
   @Override
   public boolean isFinished() {
     
     if(DriverStation.isAutonomous())
     {
       return endCommand.getAsBoolean() || noTag.getAsBoolean();
     }
     else
     {
     return false;
     }
     
   }
 }