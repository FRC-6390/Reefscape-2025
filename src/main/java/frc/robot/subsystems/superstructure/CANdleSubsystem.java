package frc.robot.subsystems.superstructure;

import com.ctre.phoenix.led.CANdle;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Superstructure;

public class CANdleSubsystem extends SubsystemBase {

  private final CANdle candle;
  private final EndEffector effector;
  private final Superstructure superstructure;

  private int red = 0, blue = 0, green = 0;
 
  public CANdleSubsystem(EndEffector effector, Superstructure superstructure) 
  {
    this.effector = effector;
    this.superstructure = superstructure;
    this.candle = new CANdle(Constants.EndEffector.CANDLE_ID, Constants.CANIVORE_CANBUS);
  }

  @Override
  public void periodic() {

    red = effector.hasGamePiece() ? 0 : 255;
    green = effector.hasGamePiece() ? 255 : 0;
    blue = superstructure.closeEnough() ? 255 : 0;

    // candle.setLEDs(255, 0, 0, 0, 0, 29);
    // candle.setLEDs(0, 0, 255, 0, 29, 100);

    candle.setLEDs(red,green,blue);
  }
}
