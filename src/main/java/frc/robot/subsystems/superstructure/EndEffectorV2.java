package frc.robot.subsystems.superstructure;

import ca.frc6390.athena.core.RobotBase;
import ca.frc6390.athena.mechanisms.Mechanism.StatefulMechanism;
import ca.frc6390.athena.mechanisms.StateMachine;
import ca.frc6390.athena.mechanisms.StateMachine.SetpointProvider;
import ca.frc6390.athena.sensors.beambreak.IRBeamBreak;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.superstructure.endeffector.AlgaeExtender;
import frc.robot.subsystems.superstructure.endeffector.AlgaeExtender.AlgaeExtenderState;
import frc.robot.subsystems.superstructure.endeffector.Rollers;
import frc.robot.subsystems.superstructure.endeffector.Rotator;
import frc.robot.subsystems.superstructure.endeffector.Rotator.RotatorState;
import frc.robot.Constants.EndEffector.PivotState;
import frc.robot.Constants.EndEffector.RollerState;


public class EndEffectorV2 extends SubsystemBase{

    private final StatefulMechanism<PivotState> joint1;
    private final StatefulMechanism<PivotState> joint2;

    private final StatefulMechanism<RollerState> rollers;

    private boolean autoEndScoring = true;

    private final StateMachine<EndEffectorTuple, EndEffectorState> stateMachine;

    private final IRBeamBreak beamBreakLeft, beamBreakRight, beamBreakCenter;

    public record EndEffectorTuple(RollerState rollerState,  PivotState joint1state, PivotState joint2state) {}
    
    public enum EndEffectorState implements SetpointProvider<EndEffectorTuple>
    {
        L4(new EndEffectorTuple(RollerState.Stopped, PivotState.Scoring, PivotState.ScoringJoint2)),
        L3(new EndEffectorTuple(RollerState.Stopped,  PivotState.Scoring, PivotState.ScoringJoint2)),
        L2(new EndEffectorTuple(RollerState.Stopped, PivotState.Scoring, PivotState.ScoringJoint2)),
        L1(new EndEffectorTuple(RollerState.Stopped, PivotState.Scoring,PivotState.ScoringJoint2)),
        Score(new EndEffectorTuple(RollerState.Scoring, PivotState.Scoring,PivotState.ScoringJoint2)),
        Stop(new EndEffectorTuple(RollerState.Stopped, null, null)),
        Home(new EndEffectorTuple(RollerState.Stopped, PivotState.Home, PivotState.HomeJoint2)),
        Intaking(new EndEffectorTuple(RollerState.Intaking, PivotState.Intaking, PivotState.IntakingJoint2));


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

   

    public EndEffectorV2( StatefulMechanism<PivotState> joint1,StatefulMechanism<PivotState> joint2, StatefulMechanism<RollerState>  Rollers ){
    
        this.joint1 = joint1;
        this.rollers = Rollers;
        this.joint2 = joint2;

        beamBreakLeft = new IRBeamBreak(1);
        beamBreakCenter = new IRBeamBreak(0);
        beamBreakRight = new IRBeamBreak(2);

        this.stateMachine = new StateMachine<EndEffectorTuple,EndEffectorState>(EndEffectorState.Home, () -> rollers.getStateMachine().atGoalState() && joint1.getStateMachine().atGoalState());
    }

    public StateMachine<EndEffectorTuple, EndEffectorState> getStateMachine() {
        return stateMachine;
    }

    public boolean hasGamePiece(){
        return beamBreakLeft.getAsBoolean() || beamBreakCenter.getAsBoolean() || beamBreakRight.getAsBoolean();
    }

    public boolean hasNoPiece(){
        return !beamBreakLeft.getAsBoolean() && !beamBreakCenter.getAsBoolean() && !beamBreakRight.getAsBoolean();
    }

    public ShuffleboardTab shuffleboard(String tab) {
        return shuffleboard(Shuffleboard.getTab(tab));
    }

    public ShuffleboardTab shuffleboard(ShuffleboardTab tab) {
       
        tab.addBoolean("BeamBreakLeft", () -> beamBreakLeft.getAsBoolean());
        tab.addBoolean("BeamBreakRight", () -> beamBreakRight.getAsBoolean());
        tab.addBoolean("BeamBreakCenter", () -> beamBreakCenter.getAsBoolean());


        return tab;
    } 

    public boolean isScoring(){
        return rollers.getStateMachine().atAnyState(RollerState.Scoring);
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

    public StatefulMechanism<PivotState> getJoint1() {
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
