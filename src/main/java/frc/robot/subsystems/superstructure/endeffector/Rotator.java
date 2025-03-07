package frc.robot.subsystems.superstructure.endeffector;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import ca.frc6390.athena.core.RobotBase;
import ca.frc6390.athena.mechanisms.StateMachine;
import ca.frc6390.athena.mechanisms.StateMachine.SetpointProvider;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.ReefScoringPos.ReefPole;

public class Rotator extends SubsystemBase {

  private final RobotBase<?> base;
  private final TalonFX motor;
  private final CANcoder encoder;
  private boolean flip = false;
  private double nudge = 0;
  private final PIDController controller;
  private final StateMachine<Double, RotatorState> stateMachine;
  private final StatusSignal<Angle> getAbsolutePosition;

  public enum RotatorState implements SetpointProvider<Double>
  {
      Home(0),
      // Algae(-16),
      Algae(0),
      L4(-38.671875000000036);

      private double angle;
      private RotatorState(double angle)
      {
        this.angle = angle;
      }

      @Override
      public Double getSetpoint() {
        return angle;
      }
  }

  public Rotator(RobotBase<?> base) 
  {
      this.base = base;
      motor = new TalonFX(Constants.EndEffector.MOTOR, Constants.EndEffector.CANBUS);
      encoder = new CANcoder(Constants.EndEffector.ENCODER, Constants.EndEffector.CANBUS);
      

      getAbsolutePosition = encoder.getAbsolutePosition();

      motor.getConfigurator().apply(new TalonFXConfiguration());
      motor.setNeutralMode(NeutralModeValue.Brake);

      CANcoderConfiguration config = new CANcoderConfiguration();
      config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;

      encoder.getConfigurator().apply(config);

      controller = Constants.EndEffector.CONTORLLER;
      controller.setTolerance(1);

      stateMachine = new StateMachine<Double, RotatorState>(RotatorState.Home, controller::atSetpoint);
  }

  public void setMotors(double speed)
  {
    motor.set(speed);
  }

  public StateMachine<Double, RotatorState> getStateMachine()
  {
    return stateMachine;
  }

  public void stopMotors() {
    motor.stopMotor();
  }

  public double getPosition() {
    return -getAbsolutePosition.getValueAsDouble();
   
  }

  public Rotation2d getAngle() {
    return Rotation2d.fromRotations(getPosition() / Constants.EndEffector.ENCODER_GEAR_RATIO).minus(Rotation2d.fromRotations(Constants.EndEffector.ENCODER_OFFSET)).minus(Rotation2d.fromDegrees(nudge));
  }

  public void setFlip(boolean shouldFlip)
  {
    flip = shouldFlip;
  }

  public void update()
  {
    
    switch (stateMachine.getGoalState()) {
      case L4:
        double setpoint = stateMachine.getGoalStateSetpoint();
        if(DriverStation.isAutonomousEnabled())
        {
         setpoint = stateMachine.getGoalStateSetpoint() * base.getCameraFacing(ReefPole.getCenterReef()).config.getYawSin();
        }
        else
        {
        setpoint = stateMachine.getGoalStateSetpoint(); //* base.getCameraFacing(ReefPole.getCenterReef()).config.getYawSin();
        }
        setMotors(controller.calculate(getAngle().getDegrees(), !flip ? setpoint : -setpoint));
      break;
      case Home:
      case Algae:
        setMotors(controller.calculate(getAngle().getDegrees(), stateMachine.getGoalState().getSetpoint()));
      break;
    }
  
  }

  public ShuffleboardLayout shuffleboard(ShuffleboardTab tab, String name) {
      return shuffleboard(tab.getLayout(name, BuiltInLayouts.kList));
  }

  public ShuffleboardLayout shuffleboard(ShuffleboardLayout tab) {
    tab.addString("State", () -> stateMachine.getGoalState().name()).withPosition(2,1);
    tab.addNumber("PID Output", () -> controller.calculate(getAngle().getDegrees(), stateMachine.getGoalState().getSetpoint())).withPosition(3,1);
    tab.addNumber("Angle", () -> getAngle().getDegrees()).withPosition(4,1);
    tab.addNumber("Rotations", this::getPosition).withPosition(4,1);
    tab.addString("Next State", () -> stateMachine.getNextState().name()).withPosition(5, 1);
    tab.addNumber("Next State Angle", () -> stateMachine.getNextState().getSetpoint()).withPosition(5, 1);
    tab.addBoolean("Is ROtator FLip", () -> flip);
    return tab;
  }

  public void refresh(){
    getAbsolutePosition.refresh();
    stateMachine.update();
  }

  @Override
  public void periodic() {
      refresh();
      update();
  }
}
