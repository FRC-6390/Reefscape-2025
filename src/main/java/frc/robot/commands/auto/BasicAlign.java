package frc.robot.commands.auto;

import ca.frc6390.athena.controllers.DelayedOutput;
import ca.frc6390.athena.core.RobotBase;
import ca.frc6390.athena.core.RobotSpeeds.SpeedAxis;
import ca.frc6390.athena.sensors.camera.limelight.LimeLight;
import ca.frc6390.athena.sensors.camera.limelight.LimeLight.PoseEstimateType;
import ca.frc6390.athena.sensors.camera.limelight.LimeLight.PoseEstimateWithLatencyType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utils.ReefScoringPos.ReefPole;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class BasicAlign extends Command {

  public LimeLight limeLight;
 
  public RobotBase<?> base;
    public MedianFilter filter;
    public DelayedOutput endCommand;
    public DelayedOutput seenTagAndLost;
    public DelayedOutput noTagFound;
    public ReefPole pole;
    public boolean hasSet = false;

  public PIDController controller = new PIDController(0.035, 0.004, 0);
  
  
  /** Creates a new PassiveAlign. */
  public BasicAlign(RobotBase<?> base, String llTable, ReefPole pole) {
    this.base = base;
    limeLight = this.base.getVision().getLimelight(llTable);
    this.pole = pole;
    hasSet = false;
    
    // Use addRequirements() here to declare subsystem dependencies.
  }

  public BasicAlign(RobotBase<?> base, String llTable) {
    this(base, llTable, ReefPole.NONE);
    hasSet = false;
    // Use addRequirements() here to declare subsystem dependencies.
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    filter = new MedianFilter(25);
    endCommand = new DelayedOutput(() -> linedUp(), 0.5);
    noTagFound = new DelayedOutput(() -> noTag(), 5);
    seenTagAndLost = new DelayedOutput(() -> hasSeenAndLost(), 1);
    controller.setIntegratorRange(-10, 10);
    hasSet = false;
    if(!pole.equals(ReefPole.NONE))
    {
      limeLight.setPriorityID((int)pole.getApriltagId());
    }
    else
    {
      limeLight.setPriorityID(-1);
    }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
  SmartDashboard.putBoolean("EndCommand",endCommand.getAsBoolean());
  SmartDashboard.putBoolean("No Tag",noTagFound.getAsBoolean());
  SmartDashboard.putNumber("Dist",limeLight.getPoseEstimate(PoseEstimateWithLatencyType.BOT_POSE_MT2_BLUE).getRaw()[9]);
  SmartDashboard.putNumber("HOFfset",limeLight.getTargetHorizontalOffset());

  if(DriverStation.isAutonomous())
  {
  if(pole.equals(ReefPole.NONE))
  {
  if(limeLight.hasValidTarget())
  {
  if(!hasSet)
  {
    hasSet = true;
  }
  if(Math.abs(filter.calculate(limeLight.getPoseEstimate(PoseEstimateType.TARGET_POSE_ROBOT_SPACE).getRaw()[4])) < 15)
    {
      double r = controller.calculate(limeLight.getTargetHorizontalOffset(), 0);
      double x = 0;
      if(base.getRobotSpeeds().getSpeeds("auto").equals(new ChassisSpeeds()))
      {
         x = 0.1;
      }
      base.getDrivetrain().getRobotSpeeds().setAxisState("auto", SpeedAxis.Y, false);
      base.getDrivetrain().getRobotSpeeds().setSpeeds("feedback", x, r, 0);
    }
  }
  else
  {
    base.getDrivetrain().getRobotSpeeds().stopSpeeds("feedback");
  }
  }
  else
  {
  if(limeLight.hasValidTarget())
  {
    if(!hasSet)
    {
      hasSet = true;
    }
  if(limeLight.getAprilTagID() == pole.getApriltagId())
  {
  if(Math.abs(filter.calculate(limeLight.getPoseEstimate(PoseEstimateType.TARGET_POSE_ROBOT_SPACE).getRaw()[4])) < 10)
    {
      double r = controller.calculate(limeLight.getTargetHorizontalOffset(), 0);
      double x = 0;
      if(base.getRobotSpeeds().getSpeeds("auto").equals(new ChassisSpeeds()))
      {
         x = 0.1;
      }
      base.getDrivetrain().getRobotSpeeds().setAxisState("auto", SpeedAxis.Y, false);
      base.getDrivetrain().getRobotSpeeds().setSpeeds("feedback", x,r,0); 
    }
  }
  else
  {
    base.getDrivetrain().getRobotSpeeds().stopSpeeds("feedback");
  }
  } 
  }
}
else
{
  if(limeLight.hasValidTarget())
  {
    if(!hasSet)
  {
    hasSet = true;
  }
  
      double r = controller.calculate(limeLight.getTargetHorizontalOffset(), 0);
      double x = 0;
      if(base.getRobotSpeeds().getSpeeds("auto").equals(new ChassisSpeeds()))
      {
         x = 0.1;
      }
      base.getDrivetrain().getRobotSpeeds().setAxisState("auto", SpeedAxis.Y, false);
      base.getDrivetrain().getRobotSpeeds().setSpeeds("feedback", x,r,0); 
  }
}
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    base.getDrivetrain().getRobotSpeeds().stopSpeeds("feedback");
    base.getDrivetrain().getRobotSpeeds().stopSpeeds("auto");

    limeLight.setPriorityID(-1);
    base.getDrivetrain().getRobotSpeeds().setAxisState("auto", SpeedAxis.Y, true);

  }

  public boolean noTag()
  {
    return !limeLight.hasValidTarget();
  }

  public Pose2d getBotPoseTagSpace(LimeLight ll)
  {
    double dist = ll.getPoseEstimate(PoseEstimateWithLatencyType.BOT_POSE_MT2_BLUE).getRaw()[9];

    // if(!pole.equals(ReefPole.NONE))
    // {
    //   offset = pole.getOffsetInDegrees();
    // }

    double angle = ll.getTargetHorizontalOffset();
    double x = (Math.cos(Math.toRadians(angle)) * dist);
    double y = (Math.sin(Math.toRadians(angle)) * dist); 
    SmartDashboard.putNumber("x",x);
    SmartDashboard.putNumber("Y", y);
    SmartDashboard.putNumber("Angle", angle);
    

    return new Pose2d(x,y, base.getLocalization().getFieldPose().getRotation());
  }

  public boolean linedUp()
  {
    Pose2d pose = getBotPoseTagSpace(limeLight);
    return limeLight.hasValidTarget() && limeLight.getPoseEstimate(PoseEstimateWithLatencyType.BOT_POSE_MT2_BLUE).getRaw()[9] <= 0.525 && Math.abs(Math.abs(limeLight.getTargetHorizontalOffset()))< 7;
  } 
  public boolean hasSeenAndLost()
  {
    return hasSet && !limeLight.hasValidTarget();
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(DriverStation.isAutonomous()) return endCommand.getAsBoolean() || noTagFound.getAsBoolean() || seenTagAndLost.getAsBoolean();
    else
    {
      return false;
    }
  }
}