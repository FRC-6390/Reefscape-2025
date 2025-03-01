package frc.robot.subsystems.superstructure.endeffector;

import com.ctre.phoenix6.hardware.TalonFX;

import ca.frc6390.athena.core.RobotBase;
import ca.frc6390.athena.mechanisms.StateMachine;
import ca.frc6390.athena.mechanisms.StateMachine.SetpointProvider;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.ReefScoringPos.ReefPole;

public class Rollers extends SubsystemBase {

  //EJECTOR STUFF
  private final TalonFX motor;
  private boolean flip = false;
  private final StateMachine<Double, RollerState> stateMachine;
  private final RobotBase<?> base;

  public enum RollerState implements SetpointProvider<Double>
  {
    
    Stopped(0),
    Algae(1),
    Running(1),
    RunningInverted(-1);

    private double speed;
    private RollerState(double angle)
    {
      this.speed = angle;
    }

    @Override
    public Double getSetpoint()
    {
      return speed;
    }
  }

  public Rollers(RobotBase<?> base) 
  {
      this.base = base;
      motor = new TalonFX(Constants.EndEffector.ROLLER, Constants.EndEffector.CANBUS);
      stateMachine = new StateMachine<Double, Rollers.RollerState>(RollerState.Stopped, () -> true);
  }

  public StateMachine<Double,RollerState> getStateMachine() {
      return stateMachine;
  }

  public void update()
  {
    
    switch (stateMachine.getGoalState()) {
      case Running:
      case Stopped:
      case RunningInverted:
            // motor.set(stateMachine.getGoalStateSetpoint());

            double setpoint = stateMachine.getGoalStateSetpoint() * base.getCameraFacing(ReefPole.getCenterReef()).config.getYawSin();
            motor.set(!flip ? setpoint : -setpoint);
        break;
      case Algae:
        motor.set(stateMachine.getGoalStateSetpoint());
      default:
        break;
    }

  }

  public void setFlip(boolean shouldFlip)
  {
    flip = shouldFlip;
  }

  public ShuffleboardLayout shuffleboard(ShuffleboardTab tab, String name) {
      return shuffleboard(tab.getLayout(name, BuiltInLayouts.kList));
  }

  public ShuffleboardLayout shuffleboard(ShuffleboardLayout tab) {
    tab.addBoolean("IsEjectorFlipped", () ->  flip).withPosition(5, 1);
    tab.addString("Next State", () -> stateMachine.getNextState().name()).withPosition(5, 1);
    return tab;
  }

  public void refresh(){
    stateMachine.update();
  }

  @Override
  public void periodic() {
      refresh();
      update();
  }
}
