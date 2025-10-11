package frc.robot.subsystems.superstructure;

import com.ctre.phoenix.led.CANdle;

import ca.frc6390.athena.core.RobotBase;
import ca.frc6390.athena.sensors.camera.LocalizationCamera;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class CANdleSubsystem extends SubsystemBase {

  private final CANdle candle;

  private int red = 0, blue = 0, green = 0;
  public RobotBase<?> base;

  public CANdleSubsystem(RobotBase<?> base)
  {
    this.base = base;;
    this.candle = new CANdle(Constants.EndEffector.CANDLE_ID, Constants.CANIVORE_CANBUS);
  }

  public void setRGB(int r, int g, int b) {
    red = r;
    blue = b;
    green = g;
  }

  public boolean closeEnough(String table) {
    LocalizationCamera ll = base.getVision().getCamera(table);
    return ll.hasValidTarget() && ll.getTargetDistanceMeters().getAsDouble() <= 0.525
        && ll.getTargetYawDegrees().getAsDouble() < 5;
  }

  public boolean noTarget(String table) {
    return !base.getVision().getCamera(table).hasValidTarget();
  }

  @Override
  public void periodic() {

    red = noTarget("limelight-left") && noTarget("limelight-right") ? 255 : 0;
    blue = (!closeEnough("limelight-left") && !closeEnough("limelight-right")) ? 255 : 0;
    green = closeEnough("limelight-left") || closeEnough("limelight-right") ? 255 : 0;
    candle.setLEDs(red, green, blue);
  }
}
