package frc.robot.subsystems.superstructure;

import ca.frc6390.athena.controllers.DelayedOutput;
import ca.frc6390.athena.mechanisms.ArmMechanism.StatefulArmMechanism;
import ca.frc6390.athena.mechanisms.Mechanism.StatefulMechanism;
import ca.frc6390.athena.mechanisms.StateMachine;
import ca.frc6390.athena.mechanisms.StateMachine.SetpointProvider;
import ca.frc6390.athena.sensors.limitswitch.GenericLimitSwitch;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.EndEffector.ArmState;
import frc.robot.Constants.EndEffector.WristState;
import frc.robot.Constants.EndEffector.RollerState;


public class EndEffector extends SubsystemBase{

    private final StatefulArmMechanism<ArmState> joint1;
    private final StatefulArmMechanism<WristState> joint2;
    // public DelayedOutput joint1AtGoal;
    // public DelayedOutput joint2AtGoal;

    private final DelayedOutput hasNoPiece;

    private final StatefulMechanism<RollerState> rollers;

    private boolean autoEndScoring = true;

    private final StateMachine<EndEffectorTuple, EndEffectorState> stateMachine;

    private final GenericLimitSwitch proximitySensor;

    private EndEffectorState prevState;

    public record EndEffectorTuple(RollerState rollerState,  ArmState joint1state, WristState joint2state) {}
    
    public enum EndEffectorState implements SetpointProvider<EndEffectorTuple>
    {
        L4(new EndEffectorTuple(RollerState.Stopped, ArmState.ScoringL4, WristState.ScoringL4)),
        L3(new EndEffectorTuple(RollerState.Stopped,  ArmState.Scoring, WristState.Scoring)),
        L2(new EndEffectorTuple(RollerState.Stopped, ArmState.Scoring, WristState.Scoring)),
        L1(new EndEffectorTuple(RollerState.Stopped, ArmState.Scoring,WristState.Scoring)),
        Score(new EndEffectorTuple(RollerState.Running, null,null)),
        Stop(new EndEffectorTuple(RollerState.Stopped, null, null)),
        Home(new EndEffectorTuple(RollerState.Stopped, ArmState.Home, WristState.Home)),
        Reverse(new EndEffectorTuple(RollerState.Reverse, null, null)),
        Intaking(new EndEffectorTuple(RollerState.Running, ArmState.Intaking, WristState.Intaking)),
        Transition(new EndEffectorTuple(RollerState.Stopped, ArmState.TransitionState, WristState.TransitionState));



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

   

    public EndEffector( StatefulArmMechanism<ArmState> joint1, StatefulArmMechanism<WristState> joint2, StatefulMechanism<RollerState> Rollers ){
    
        this.joint1 = joint1;
        this.rollers = Rollers;
        this.joint2 = joint2;    
        this.prevState = EndEffectorState.Home;

        proximitySensor = new GenericLimitSwitch(4, true);
        hasNoPiece = new DelayedOutput(() -> !hasGamePiece(), 0.25);

        this.stateMachine = new StateMachine<EndEffectorTuple,EndEffectorState>(EndEffectorState.Home, () -> joint1.getStateMachine().atGoalState()&& joint2.getStateMachine().atGoalState());
    }

    public StateMachine<EndEffectorTuple, EndEffectorState> getStateMachine() {
        return stateMachine;
    }

    public boolean hasGamePiece(){
        return proximitySensor.getAsBoolean();
    }

    public boolean hasNoPiece(){
        return hasNoPiece.getAsBoolean();
    }

    public ShuffleboardTab shuffleboard(String tab) {
        return shuffleboard(Shuffleboard.getTab(tab));
    }

    public ShuffleboardTab shuffleboard(ShuffleboardTab tab) {
       
        tab.addBoolean("Has Game Piece", () -> hasGamePiece());


        return tab;
    } 

    public boolean isScoring(){
        return rollers.getStateMachine().atAnyState(RollerState.Running) && !joint1.getStateMachine().getGoalState().equals(ArmState.Intaking);
    }

    public EndEffector setAutoEndScoring(boolean autoEndScoring) {
        this.autoEndScoring = autoEndScoring;
        return this;
    }


    public void update(){

        stateMachine.update();
        rollers.update();
        joint1.update();

        EndEffectorState state = stateMachine.getGoalState();
        EndEffectorTuple val = stateMachine.getGoalStateSetpoint();

        if(!prevState.equals(state)){
            switch (state) {                  
                default:
                    if (val.rollerState != null) rollers.getStateMachine().queueState(val.rollerState);
                    if (val.joint1state != null) joint1.getStateMachine().queueState(val.joint1state);
                    if (val.joint2state != null) joint2.getStateMachine().queueState(val.joint2state);
                break;
            }
        }

        prevState = state;
        
    }


    public StatefulMechanism<RollerState> getRollers() {
        return rollers;
    }

    public StatefulArmMechanism<ArmState> getJoint1() {
        return joint1;
    }
    public StatefulArmMechanism<WristState> getJoint2() {
        return joint2;
    }

    public boolean isAutoEndScoring() {
        return autoEndScoring;
    }


    @Override
    public void periodic() {
        update();
    }
}
