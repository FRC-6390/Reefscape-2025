package frc.robot.subsystems.superstructure.endeffector;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import ca.frc6390.athena.mechanisms.StateMachine;
import ca.frc6390.athena.mechanisms.StateMachine.SetpointProvider;
import ca.frc6390.athena.sensors.limitswitch.GenericLimitSwitch;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AlgaeExtender extends SubsystemBase {

  public final TalonFX motor;
  public final GenericLimitSwitch limitSwitchAlgae;
  public final PIDController controller;
  public final StateMachine<Double, AlgaeExtenderState> stateMachine;
  public final StatusSignal<Angle> getPosition;

  public enum AlgaeExtenderState implements SetpointProvider<Double>
  {
    Extended(-2.3),
    Home(0);

    private double angle;
    private AlgaeExtenderState(double angle)
    {
      this.angle = angle;
    }

    @Override
    public Double getSetpoint()
    {
      return angle;
    }
  }

  public AlgaeExtender() 
  {
      limitSwitchAlgae = new GenericLimitSwitch(Constants.EndEffector.LIMIT_SWITCH);
      motor = new TalonFX(Constants.EndEffector.ALGAE_MOTOR, Constants.EndEffector.CANBUS);

      getPosition = motor.getRotorPosition();

      motor.getConfigurator().apply(new TalonFXConfiguration());
      motor.setNeutralMode(NeutralModeValue.Brake);
      CurrentLimitsConfigs limit = new CurrentLimitsConfigs();
      limit = limit.withStatorCurrentLimit(30);
      motor.getConfigurator().apply(limit);
      controller = new PIDController(1, 0, 0);
      controller.setTolerance(1);
      stateMachine = new StateMachine<Double, AlgaeExtenderState>(AlgaeExtenderState.Home, this::atSetpoint);
      motor.setPosition(0);
      limitSwitchAlgae.whileTrue(() -> motor.setPosition(AlgaeExtenderState.Extended.getSetpoint()));
  }

  public void setMotors(double speed)
  {
    motor.set(speed);
  }

  public StateMachine<Double, AlgaeExtenderState> getStateMachine()
  {
    return stateMachine;
  }

  public void stopMotors() {
    motor.stopMotor();
  }

  public boolean atSetpoint()
  {
    if(stateMachine.getGoalState().equals(AlgaeExtenderState.Home)) 
    {
    return controller.atSetpoint();
    }
    else
    {
      return controller.atSetpoint() && limitSwitchAlgae.getAsBoolean();
    }
  }

  public void setPostion(double rotations)
  {
    motor.setPosition(0);
  }

  public void update()
  {
    // if(limitSwitchAlgae.getAsBoolean())
    // {
    //   motor.setPosition(AlgaeExtenderState.Extended.getSetpoint());
    // }

    // switch (stateMachine.getGoalState()) 
    // {
    //   case Home:
    //     setMotors(-controller.calculate(getPosition.getValueAsDouble(),AlgaeExtenderState.Home.getSetpoint()));
    //     break; 
    //   case Extended:
    //     double spd = Math.abs(controller.calculate(getPosition.getValueAsDouble(),AlgaeExtenderState.Extended.getSetpoint()));
    //     setMotors(spd + 0.3);
    //   default:
    //     break;
    // }
  }


  public ShuffleboardLayout shuffleboard(ShuffleboardTab tab, String name) {
      return shuffleboard(tab.getLayout(name, BuiltInLayouts.kList));
  }


  public ShuffleboardLayout shuffleboard(ShuffleboardLayout tab) {
    tab.addBoolean("Limit Switch", limitSwitchAlgae::getAsBoolean).withPosition(1,1);
    tab.addString("State", () -> stateMachine.getGoalState().name()).withPosition(2,1);
    tab.addString("Next State", () -> stateMachine.getNextState().name()).withPosition(5, 1);
    tab.addNumber("Next State Angle", () -> stateMachine.getNextState().getSetpoint()).withPosition(5, 1);
    tab.addNumber("Position", () -> getPosition.getValueAsDouble()).withPosition(5, 1);
    tab.addNumber("PID output", () -> controller.calculate(AlgaeExtenderState.Extended.getSetpoint())).withPosition(5, 1);
    return tab;
  }

  public void refresh(){
    getPosition.refresh();
    stateMachine.update();
  }

  @Override
  public void periodic() {
      refresh();
      update();
  }
}
