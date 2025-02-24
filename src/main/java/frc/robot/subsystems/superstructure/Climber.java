package frc.robot.subsystems.superstructure;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import ca.frc6390.athena.mechanisms.StateMachine;
import ca.frc6390.athena.mechanisms.StateMachine.SetpointProvider;
import ca.frc6390.athena.sensors.limitswitch.GenericLimitSwitch;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
public class Climber extends SubsystemBase{
  // public TalonFX leftMotor;
  // public TalonFX rightMotor;
  // public CANcoder encoder;
  
  // public GenericLimitSwitch limitSwitch;
  public PIDController controller;

  public StateMachine<ClimberState> stateMachine;
  // public StatusSignal<Angle> getAbsolutePosition = new StatusSignal<>(null, null, null);

  public enum ClimberState implements SetpointProvider
  {
      Home(0),
      Climb(45);
      private double angle;
      private ClimberState(double angle)
      {
      this.angle = angle;
      }

      @Override
      public double getSetpoint() {
        return angle;
      }
  }
  /** Creates a new Climber. */
  public Climber() 
  {
    // leftMotor = new TalonFX(Constants.Climber.LEFT_MOTOR, Constants.Climber.CANBUS);
    // rightMotor = new TalonFX(Constants.Climber.RIGHT_MOTOR, Constants.Climber.CANBUS);
    // encoder = new CANcoder(Constants.Climber.ENCODER, Constants.Climber.CANBUS);
    // limitSwitch = new GenericLimitSwitch(Constants.Climber.LIMIT_SWITCH);

    // getAbsolutePosition = encoder.getAbsolutePosition();

    controller = Constants.Climber.CONTORLLER;
    controller.enableContinuousInput(0, 90);
    controller.setTolerance(1);

    // leftMotor.getConfigurator().apply(new TalonFXConfiguration());
    // rightMotor.getConfigurator().apply(new TalonFXConfiguration());

    // leftMotor.setNeutralMode(NeutralModeValue.Brake);
    // rightMotor.setNeutralMode(NeutralModeValue.Brake);

    CANcoderConfiguration config = new CANcoderConfiguration();
    config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
    // encoder.getConfigurator().apply(config);

    stateMachine = new StateMachine<ClimberState>(ClimberState.Home, this::test);
  }

  public boolean test()
  {
    return true;
  }
  public void setMotors(double speed)
  {
    // if(limitSwitch.getAsBoolean() && speed > 0)
    {
      // speed = 0;
    }
    // leftMotor.set(speed);
    // rightMotor.set(-speed);
  }

  public StateMachine<ClimberState> getStateMachine()
  {
    return stateMachine;
  }

  public void stopMotors() {
    // leftMotor.stopMotor();
    // rightMotor.stopMotor();
  }

  public void update()
  {
    switch (stateMachine.getGoalState()) {
      case Climb, Home:
      double speed = -controller.calculate(getAngle().getDegrees(), stateMachine.getGoalState().getSetpoint());
      setMotors(speed);
    }
  }

  public ShuffleboardTab shuffleboard(String tab) {
      return shuffleboard(Shuffleboard.getTab(tab));
  }

  public ShuffleboardTab shuffleboard(ShuffleboardTab tab) {
    // tab.addBoolean("Limit Switch", limitSwitch::getAsBoolean).withPosition(1,1);
    tab.addString("Setpoint", () -> stateMachine.getGoalState().name()).withPosition(2,1);
    tab.addNumber("PID Output", () -> controller.calculate(getAngle().getDegrees(), stateMachine.getGoalState().getSetpoint())).withPosition(3,1);
    tab.addNumber("Angle", () -> getAngle().getDegrees()).withPosition(4,1);
    tab.addNumber("Rotations", this::getPosition).withPosition(5,1);
    tab.addString("Next State", () -> stateMachine.getNextState().name()).withPosition(6, 1);

    return tab;
  }

  public double getPosition() {
    return 0;
    // return getAbsolutePosition.getValueAsDouble();
  }

  public Rotation2d getAngle() {
    return Rotation2d.fromRotations(getPosition() / Constants.Climber.ENCODER_GEAR_RATIO).minus(Rotation2d.fromDegrees(Constants.Climber.ENCODER_OFFSET));
  }

  public void refresh(){
    // getAbsolutePosition.refresh();
    stateMachine.update();
  }

  @Override
  public void periodic() {
      refresh();
      update();
  }
}
