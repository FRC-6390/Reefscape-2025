package frc.robot.subsystems.superstructure;

import ca.frc6390.athena.commands.RunnableTrigger;
import ca.frc6390.athena.core.RobotBase;
import ca.frc6390.athena.mechanisms.ArmMechanism.StatefulArmMechanism;
import ca.frc6390.athena.mechanisms.Mechanism.StatefulMechanism;
import ca.frc6390.athena.mechanisms.Mechanism;
import ca.frc6390.athena.mechanisms.StateMachine;
import ca.frc6390.athena.mechanisms.StateMachine.SetpointProvider;
import ca.frc6390.athena.sensors.beambreak.IRBeamBreak;
import ca.frc6390.athena.sensors.limitswitch.GenericLimitSwitch;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.superstructure.endeffector.AlgaeExtender;
import frc.robot.subsystems.superstructure.endeffector.AlgaeExtender.AlgaeExtenderState;
// import frc.robot.subsystems.superstructure.endeffector.Rollers;
import frc.robot.subsystems.superstructure.endeffector.Rotator;
import frc.robot.subsystems.superstructure.endeffector.Rotator.RotatorState;
import frc.robot.Constants.EndEffector.ArmState;
import frc.robot.Constants.EndEffector.WristState;
import frc.robot.Constants.EndEffector.RollerState;


public class EndEffectorV2 extends SubsystemBase{

    private final StatefulArmMechanism<ArmState> joint1;
    private final StatefulArmMechanism<WristState> joint2;
    private RunnableTrigger liftIntake;

    private final StatefulMechanism<RollerState> rollers;

    private boolean autoEndScoring = true;

    private final StateMachine<EndEffectorTuple, EndEffectorState> stateMachine;

    private final GenericLimitSwitch proximitySensor;

    public record EndEffectorTuple(RollerState rollerState,  ArmState joint1state, WristState joint2state) {}
    
    public enum EndEffectorState implements SetpointProvider<EndEffectorTuple>
    {
        L4(new EndEffectorTuple(RollerState.Stopped, ArmState.ScoringL4, WristState.ScoringL4)),
        L3(new EndEffectorTuple(RollerState.Stopped,  ArmState.Scoring, WristState.Scoring)),
        L2(new EndEffectorTuple(RollerState.Stopped, ArmState.Scoring, WristState.Scoring)),
        L1(new EndEffectorTuple(RollerState.Stopped, ArmState.Scoring,WristState.Scoring)),
        Score(new EndEffectorTuple(RollerState.Running, ArmState.Scoring,WristState.Scoring)),
        Stop(new EndEffectorTuple(RollerState.Stopped, null, null)),
        Home(new EndEffectorTuple(RollerState.Stopped, ArmState.Home, WristState.Home)),
        Reverse(new EndEffectorTuple(RollerState.Reverse, null, null)),
        Intaking(new EndEffectorTuple(RollerState.Running, ArmState.Intaking, WristState.Intaking));


        private EndEffectorTuple states;
        private EndEffectorState(EndEffectorTuple states)
        {
            this.states = states;
        }

        @Override
        public EndEffectorTuple getSetpoint()
        {
            return states;
        }
    }

   

    public EndEffectorV2( StatefulArmMechanism<ArmState> joint1, StatefulArmMechanism<WristState> joint2, StatefulMechanism<RollerState> Rollers ){
    
        this.joint1 = joint1;
        this.rollers = Rollers;
        this.joint2 = joint2;    

        proximitySensor = new GenericLimitSwitch(4, true);

        this.stateMachine = new StateMachine<EndEffectorTuple,EndEffectorState>(EndEffectorState.Home, () -> joint1.getStateMachine().atGoalState()&& joint2.getStateMachine().atGoalState());
        proximitySensor.onTrue(() -> stateMachine.setGoalState(EndEffectorState.L1));
        proximitySensor.onFalse(() -> stateMachine.setGoalState(EndEffectorState.L1));
        // proximitySensor.onTrue(() -> stateMachine.setGoalState(EndEffectorState.Reverse)).after(0.1).onTrue(() ->stateMachine.setGoalState(EndEffectorState.Home));
    }

    public StateMachine<EndEffectorTuple, EndEffectorState> getStateMachine() {
        return stateMachine;
    }

    public boolean hasGamePiece(){
        return proximitySensor.getAsBoolean();
    }

    public ShuffleboardTab shuffleboard(String tab) {
        return shuffleboard(Shuffleboard.getTab(tab));
    }

    public ShuffleboardTab shuffleboard(ShuffleboardTab tab) {
       
        tab.addBoolean("Has Game Piece", () -> hasGamePiece());


        return tab;
    } 

    public boolean isScoring(){
        return rollers.getStateMachine().atAnyState(RollerState.Running);
    }

    public EndEffectorV2 setAutoEndScoring(boolean autoEndScoring) {
        this.autoEndScoring = autoEndScoring;
        return this;
    }


    public void update(){

        stateMachine.update();
        rollers.update();
        joint1.update();

        EndEffectorState state = stateMachine.getGoalState();
        EndEffectorTuple val = stateMachine.getGoalStateSetpoint();
        switch (state) {
            case L4: 
            case L3: 
            case L2: 
            case L1:  
            case Home:
            case Score:
            case Intaking:
            case Stop:
                if (val.rollerState != null) rollers.getStateMachine().setGoalState(val.rollerState);
                if (val.joint1state != null) joint1.getStateMachine().setGoalState(val.joint1state);
                if (val.joint2state != null) joint2.getStateMachine().setGoalState(val.joint2state);
            default:
                break;
        }
    }


    public StatefulMechanism<RollerState> getRollers() {
        return rollers;
    }

    public StatefulArmMechanism<ArmState> getJoint1() {
        return joint1;
    }

    public boolean isAutoEndScoring() {
        return autoEndScoring;
    }


    @Override
    public void periodic() {
        update();
    }
}
