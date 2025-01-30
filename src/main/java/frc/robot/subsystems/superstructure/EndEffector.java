package frc.robot.subsystems.superstructure;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import ca.frc6390.athena.mechanisms.StateMachine;
import ca.frc6390.athena.sensors.limitswitch.GenericLimitSwitch;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class EndEffector extends SubsystemBase {
  public TalonFX leftMotor;
  public TalonFX rightMotor;
  public CANcoder encoder;
  public GenericLimitSwitch limitSwitch;
  public PIDController controller;

  public StateMachine<State> stateMachine;

  public enum State
  {
      StartConfiguration(35),
      Home(0),
      Left(35),
      Right(-35),
      LeftL4(35),
      RightL4(-35);

      private double angle;
      private State(double angle)
      {
        this.angle = angle;
      }

      public Double getValue() {
        return angle;
      }
  }
  /** Creates a new Climber. */
  public EndEffector() 
  {
    leftMotor = new TalonFX(Constants.Climber.LEFT_MOTOR, Constants.Climber.CANBUS);
    rightMotor = new TalonFX(Constants.Climber.RIGHT_MOTOR, Constants.Climber.CANBUS);
    encoder = new CANcoder(Constants.Climber.ENCODER, Constants.Climber.CANBUS);
    limitSwitch = new GenericLimitSwitch(Constants.Climber.LIMIT_SWITCH);

    controller = Constants.Climber.controller;
    controller.enableContinuousInput(0, 90);
    controller.setTolerance(1);

    leftMotor.getConfigurator().apply(new TalonFXConfiguration());
    rightMotor.getConfigurator().apply(new TalonFXConfiguration());

    leftMotor.setNeutralMode(NeutralModeValue.Brake);
    rightMotor.setNeutralMode(NeutralModeValue.Brake);

    CANcoderConfiguration config = new CANcoderConfiguration();
    config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
    encoder.getConfigurator().apply(config);

    stateMachine = new StateMachine<State>(State.Home, controller::atSetpoint);
  }

  public void setMotors(double speed)
  {
    if(limitSwitch.isPressed() && speed > 0)
    {
      speed = 0;
    }
    leftMotor.set(speed);
    rightMotor.set(-speed);
  }

  public StateMachine<State> getStateMachine()
  {
    return stateMachine;
  }

  public void stopMotors() {
    leftMotor.stopMotor();
    rightMotor.stopMotor();
  }

  public double getPosition() {
    return encoder.getAbsolutePosition(true).getValueAsDouble();
  }

  public Rotation2d getAngle() {
    return Rotation2d.fromRotations(getPosition() / Constants.Climber.ENCODER_GEAR_RATIO).minus(Rotation2d.fromDegrees(Constants.Climber.ENCODER_OFFSET));
  }

  public void update()
  {
    shuffleboard();
    switch (stateMachine.getGoalState()) {
      case Left, Right, RightL4, LeftL4, Home, StartConfiguration:
      double speed = -controller.calculate(getAngle().getDegrees(), (double)stateMachine.getGoalState().getValue());
      setMotors(speed);
    }
  }

  public void shuffleboard()
  {
    SmartDashboard.putBoolean("Limit Switch", limitSwitch.isPressed());
    SmartDashboard.putNumber("Setpoint", stateMachine.getGoalState().getValue());
    SmartDashboard.putNumber("PID Output", controller.calculate(getAngle().getDegrees(), (double)stateMachine.getGoalState().getValue()));
    SmartDashboard.putNumber("Angle", getAngle().getDegrees());
    SmartDashboard.putNumber("Rotations", getPosition());
  }

  @Override
  public void periodic() {
      update();
  }
}
