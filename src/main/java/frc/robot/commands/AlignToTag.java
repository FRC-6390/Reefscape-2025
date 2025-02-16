package frc.robot.commands;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.function.Function;
import java.util.stream.Collector;
import java.util.stream.Collectors;
import java.util.stream.LongStream;

import ca.frc6390.athena.core.RobotBase;
import ca.frc6390.athena.core.RobotSpeeds;
import ca.frc6390.athena.drivetrains.swerve.SwerveDrivetrain;
import ca.frc6390.athena.sensors.camera.limelight.LimeLight;
import ca.frc6390.athena.sensors.camera.limelight.LimeLight.PoseEstimate;
import ca.frc6390.athena.sensors.camera.limelight.LimeLight.PoseEstimateType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.drive.RobotDriveBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class AlignToTag extends Command{
    
    public enum AlignMode{
        X,
        Y,
        THETA
    }

    private final ArrayList<Long> idFilter;
    private final AlignMode mode;
    private final RobotSpeeds speeds;
    private final LimeLight camera;

    public AlignToTag(RobotBase<SwerveDrivetrain> base, String camera, AlignMode mode, long... idFilter){
        this(base.getDrivetrain().getRobotSpeeds(), base.getVision().getCamera(camera), mode, idFilter);
    }

    public AlignToTag(RobotSpeeds speeds, LimeLight camera, AlignMode mode, long... idFilter){
        this.speeds = speeds;
        this.camera = camera;
        this.mode = mode;
        this.idFilter = new ArrayList<>(LongStream.of(idFilter).boxed().collect(Collectors.toList()));
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        if(!camera.hasValidTarget()) return;
        if (idFilter.isEmpty()) idFilter.add(camera.getAprilTagID());
        if (!idFilter.contains(camera.getAprilTagID())) return;

        PoseEstimate poseEstimate = camera.getPoseEstimate(PoseEstimateType.TARGET_POSE_ROBOT_SPACE);
        Pose2d pose = poseEstimate.getPose();
        speeds.setFeedbackSpeeds(, 0, 0);
    }

    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean isFinished() {
       return false;
    }

    
    
}
