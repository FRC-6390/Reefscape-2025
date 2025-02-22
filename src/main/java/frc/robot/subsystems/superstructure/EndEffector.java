package frc.robot.subsystems.superstructure;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import ca.frc6390.athena.mechanisms.StateMachine;
import ca.frc6390.athena.mechanisms.StateMachine.SetpointProvider;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class EndEffector extends SubsystemBase {
  // public TalonFX motor;
  public TalonFX roller;
  // public CANcoder encoder;
  public PIDController controller;
  public double rollerSpeed = 0;

  public StateMachine<EndEffectorState> stateMachine;
  public StatusSignal<Angle> getAbsolutePosition = new StatusSignal<>(null, null, null);

  public enum EndEffectorState implements SetpointProvider
  {
      //STARTCONFIG(35)
      StartConfiguration(0),
      Home(0),
      Left(35),
      //-35
      Right(-35),
      LeftL4(35),
      RightL4(-35);

      private double angle;
      private EndEffectorState(double angle)
      {
        this.angle = angle;
      }

      @Override
      public double getSetpoint() {
        return angle;
      }
  }

  /** Creates a new EndEffector. */
  public EndEffector() 
  {
    // motor = new TalonFX(Constants.EndEffector.MOTOR, Constants.EndEffector.CANBUS);
    roller = new TalonFX(Constants.EndEffector.ROLLER, Constants.EndEffector.CANBUS);
    // encoder = new CANcoder(Constants.EndEffector.ENCODER, Constants.EndEffector.CANBUS);

    // getAbsolutePosition = encoder.getAbsolutePosition();

    controller = Constants.EndEffector.CONTORLLER;
    controller.enableContinuousInput(0, 90);
    controller.setTolerance(1);

    // motor.getConfigurator().apply(new TalonFXConfiguration());

    // motor.setNeutralMode(NeutralModeValue.Brake);

    CANcoderConfiguration config = new CANcoderConfiguration();
    config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
    
    // encoder.getConfigurator().apply(config);

    stateMachine = new StateMachine<EndEffectorState>(EndEffectorState.Home, this::test);
  }

  public boolean test()
  {
    return true;
  }

  public void setMotors(double speed)
  {
    // motor.set(speed);
  }


  public void setRollers(double speed)
  {
    rollerSpeed = speed;
  }
  public StateMachine<EndEffectorState> getStateMachine()
  {
    return stateMachine;
  }

  public void stopMotors() {
    // motor.stopMotor();
  }

  public double getPosition() {
    return getAbsolutePosition.getValueAsDouble();
  }

  public Rotation2d getAngle() {
    return Rotation2d.fromRotations(getPosition() / Constants.EndEffector.ENCODER_GEAR_RATIO).minus(Rotation2d.fromDegrees(Constants.EndEffector.ENCODER_OFFSET));
  }

  public void update()
  {
    switch (stateMachine.getGoalState()) {
      case Left, Right, RightL4, LeftL4, Home, StartConfiguration:
      double speed = -controller.calculate(getAngle().getDegrees(), stateMachine.getGoalState().getSetpoint());
      setMotors(speed);
    }
    roller.set(rollerSpeed);
  }

  public ShuffleboardTab shuffleboard(String tab) {
      return shuffleboard(Shuffleboard.getTab(tab));
  }

  public ShuffleboardTab shuffleboard(ShuffleboardTab tab) {
    // tab.addBoolean("Limit Switch", limitSwitch::isPressed).withPosition(1,1);
    tab.addString("State", () -> stateMachine.getGoalState().name()).withPosition(2,1);
    // tab.addNumber("PID Output", () -> controller.calculate(getAngle().getDegrees(), stateMachine.getGoalState().getSetpoint())).withPosition(3,1);
    // tab.addNumber("Angle", () -> getAngle().getDegrees()).withPosition(4,1);
    tab.addNumber("Rotations", this::getPosition).withPosition(4,1);
    // tab.addString("Next State", () -> stateMachine.getNextState().name()).withPosition(5, 1);

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
