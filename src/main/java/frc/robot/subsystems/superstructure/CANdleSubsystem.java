package frc.robot.subsystems.superstructure;

import com.ctre.phoenix.led.CANdle;

import ca.frc6390.athena.core.RobotBase;
import ca.frc6390.athena.sensors.camera.limelight.LimeLight;
import ca.frc6390.athena.sensors.camera.limelight.LimeLight.PoseEstimateWithLatencyType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.ReefScoringPos.ReefPole;

public class CANdleSubsystem extends SubsystemBase {

  private final CANdle candle;
  // private final EndEffector effector;
  // private final Superstructure superstructure;

  private int red = 0, blue = 0, green = 0;
  public RobotBase<?> base;
 
  public CANdleSubsystem(RobotBase<?> base)//EndEffector effector, Superstructure superstructure) 
  {
    // this.effector = effector;
    this.base = base;
    // this.superstructure = superstructure;
    this.candle = new CANdle(Constants.EndEffector.CANDLE_ID, Constants.CANIVORE_CANBUS);
  }

  public void setRGB(int r, int g, int b)
  {
    red = r;
    blue = b;
    green = g;
  }

  //   public boolean closeEnough()
  // {
    public boolean closeEnough(String table)
    {
      LimeLight ll = base.getVision().getLimelight(table);
      return ll.hasValidTarget() && ll.getPoseEstimate(PoseEstimateWithLatencyType.BOT_POSE_MT2_BLUE).getRaw()[9] <= 0.525 && Math.abs(ll.getTargetHorizontalOffset()) < 5;
    }

    public boolean noTarget(String table)
    {
      LimeLight ll = base.getVision().getLimelight(table);
      return !ll.hasValidTarget(); 
    }
   
    // LimeLight ll = base.getVision().getLimelight("limelight-left");
    // LimeLight lr = base.getVision().getLimelight("limelight-right");
    
    // double dist = 99;
    // double dist2 = 99;
    // if(ll != null)
    // {
    //     ReefPole pole = ReefPole.getPoleFromID(ll.getAprilTagID(), ll);
    //     if(pole != null && ll.hasValidTarget())
    //     {
    //       dist = ll.getTargetHorizontalOffset();
    //     }
    // }

    // if(lr != null)
    // {
    //     ReefPole pole = ReefPole.getPoleFromID(lr.getAprilTagID(), lr);
    //     if(pole != null && lr.hasValidTarget())
    //     {
    //       dist2 = lr.getTargetHorizontalOffset();
    //     }
    // }

    // return Math.abs(dist) < 5.5 || Math.abs(dist2) < 5.5;
// }


  @Override
  public void periodic() {

    red = noTarget("limelight-left") && noTarget("limelight-right") ? 255 : 0;
    blue =  (!closeEnough("limelight-left") && !closeEnough("limelight-right"))? 255 : 0;
    green = closeEnough("limelight-left") || closeEnough("limelight-right") ? 255 : 0;

    // candle.setLEDs(255, 0, 0, 0, 0, 29);
    // candle.setLEDs(0, 0, 255, 0, 29, 100);

    candle.setLEDs(red, green,blue);
  }
}
