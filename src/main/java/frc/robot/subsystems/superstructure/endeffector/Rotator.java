package frc.robot.subsystems.superstructure.endeffector;

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
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Rotator extends SubsystemBase {

  public final TalonFX motor;
  public final CANcoder encoder;
  public double nudge = 0;
  public final PIDController controller;
  public final StateMachine<Double, RotatorState> stateMachine;
  public final StatusSignal<Angle> getAbsolutePosition;

  public enum RotatorState implements SetpointProvider<Double>
  {
      StartConfiguration(0),
      Home(0),
      Left(0),
      Right(0),
      LeftL4(20),
      RightL4(-20);

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

  public Rotator() 
  {
    //ROTATOR STUFF
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

  public void update()
  {
    switch (stateMachine.getGoalState()) {
      case Left, Right, RightL4, LeftL4, Home, StartConfiguration:
      double speed = controller.calculate(getAngle().getDegrees(), stateMachine.getGoalState().getSetpoint());
      setMotors(speed);
    }
  }


  public ShuffleboardLayout shuffleboard(ShuffleboardTab tab, String name) {
      return shuffleboard(tab.getLayout(name));
  }

  public ShuffleboardLayout shuffleboard(ShuffleboardLayout tab) {
    tab.addString("State", () -> stateMachine.getGoalState().name()).withPosition(2,1);
    tab.addNumber("PID Output", () -> controller.calculate(getAngle().getDegrees(), stateMachine.getGoalState().getSetpoint())).withPosition(3,1);
    tab.addNumber("Angle", () -> getAngle().getDegrees()).withPosition(4,1);
    tab.addNumber("Rotations", this::getPosition).withPosition(4,1);
    tab.addString("Next State", () -> stateMachine.getNextState().name()).withPosition(5, 1);
    tab.addNumber("Next State Angle", () -> stateMachine.getNextState().getSetpoint()).withPosition(5, 1);

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
