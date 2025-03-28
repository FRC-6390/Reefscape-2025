package frc.robot.commands.auto;
 
import ca.frc6390.athena.controllers.DelayedOutput;
 import ca.frc6390.athena.core.RobotBase;
import ca.frc6390.athena.core.RobotSpeeds;
import ca.frc6390.athena.core.RobotVision;
import ca.frc6390.athena.core.RobotSpeeds.SpeedAxis;
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
 
 public class AlgaeAlign extends Command {
   public RobotSpeeds robotSpeeds;
   public RobotVision vision;
   public LimeLight ll;
   public long runTag;
   public Translation2d curTranslation;
   public MedianFilter filter;
   public double thetaMeasurement;
   public ProfiledPIDController xController = new ProfiledPIDController(1, 0, 0, new Constraints(1, 1));
   public ProfiledPIDController yController =  new ProfiledPIDController(3.85, 0, 0, new Constraints(2, 2));
   public PIDController rController = new PIDController(0.025, 0, 0);
   public DelayedOutput endCommand;
   public Superstructure superstructure;
   public DelayedOutput noTag;
   public SuperstructureState state;

   public AlgaeAlign(RobotBase<?> base, Superstructure superstructure, SuperstructureState state)
   {
    this.superstructure = superstructure;
    this.state = state;
    this.robotSpeeds = base.getDrivetrain().getRobotSpeeds();
    this.vision = base.getVision();
   }


   @Override
   public void initialize() 
   {
    var templl = vision.getLimelights().values().stream().filter(ll -> ll.hasValidTarget()).findFirst();
    if(templl.isPresent()){
      ll = templl.get();
     }else{
      this.cancel();
     }
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
       xController.setP(1.5);
     }
     else
     {
      if(DriverStation.isTeleop() || DriverStation.isTeleopEnabled())
      {
       xController.setP(0.75);
      }
      else if(DriverStation.isAutonomous() || DriverStation.isAutonomousEnabled())
      {
        xController.setP(0.5);
      }
     }
 
    
 
     if(!ll.hasValidTarget())
     {
        robotSpeeds.setAxisState("drive",SpeedAxis.Y, true);
        robotSpeeds.stopSpeeds("feedback");
        return;
     }

  
       double Xspeed = -xController.calculate(curTranslation.getX(), Units.inchesToMeters(25));
       double YSpeed = yController.calculate(curTranslation.getY(),0);
       double rSpeed = rController.calculate(thetaMeasurement, 0);//* Math.copySign(1, ll.getTargetHorizontalOffset()), 0);

       if(ll.getPoseEstimate(PoseEstimateWithLatencyType.BOT_POSE_MT2_BLUE).getRaw()[9] <= 100000000)
       {
         SmartDashboard.putBoolean("ALGIN TAKEN",true);
         robotSpeeds.setSpeeds("feedback",Xspeed, YSpeed, -rSpeed);
       }
       else
       {
         SmartDashboard.putBoolean("ALGIN TAKEN",false);
         robotSpeeds.stopSpeeds("feedback");
       }


    if(DriverStation.isTeleop())
    {

     if(endCommand.getAsBoolean())
     {
      
      robotSpeeds.setAxisState("drive",SpeedAxis.Y, false);

      if(!superstructure.stateMachine.getGoalState().equals(state))
      {
      
      if(!superstructure.getStateMachine().atState(SuperstructureState.AlgaeHigh,SuperstructureState.AlgaeLow))
      {
        superstructure.setSuper(state);
      }
      }
     }else {
      robotSpeeds.setAxisState("drive",SpeedAxis.Y, true);
     }
    }
    
   }
 
   public boolean closeEnough()
   {
     return ll.hasValidTarget() && Math.abs(getOffsetToTarget()) < 5;
   }

   public double getOffsetToTarget(){
      return ll.getTargetHorizontalOffset() + 15;
   }
 
   // Called once the command ends or is interrupted.
   @Override
   public void end(boolean interrupted) {
     
     
    robotSpeeds.stopSpeeds("feedback");
    System.out.println("ENDED");
    superstructure.setSuper(SuperstructureState.HomePID);
    robotSpeeds.setAxisState("drive",SpeedAxis.Y, true);

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