// package frc.robot.commands;

// import java.util.ArrayList;
// import java.util.stream.Collectors;
// import java.util.stream.LongStream;

// import ca.frc6390.athena.core.RobotBase;
// import ca.frc6390.athena.core.RobotSpeeds;
// import ca.frc6390.athena.drivetrains.swerve.SwerveDrivetrain;
// import ca.frc6390.athena.filters.FilteredPose;
// import ca.frc6390.athena.sensors.camera.limelight.LimeLight;
// import ca.frc6390.athena.sensors.camera.limelight.LimeLight.PoseEstimateType;
// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
// import edu.wpi.first.wpilibj2.command.Command;

// public class AlignToTag extends Command{
    
//     public enum AlignMode{
//         X,
//         Y,
//         THETA
//     }

//     private final ArrayList<Long> idFilter;
//     // private final AlignMode mode;
//     private final RobotSpeeds speeds;
//     private final LimeLight camera;
//     private final FilteredPose pose;
//     private final ProfiledPIDController xController, yController, thetaController;


//     public AlignToTag(RobotBase<SwerveDrivetrain> base, String camera, long... idFilter){
//         this(base.getDrivetrain().getRobotSpeeds(), base.getVision().getCamera(camera), idFilter);
//     }

//     public AlignToTag(RobotSpeeds speeds, LimeLight camera, long... idFilter){
//         this.speeds = speeds;
//         this.camera = camera;
//         this.idFilter = new ArrayList<>(LongStream.of(idFilter).boxed().collect(Collectors.toList()));
//         this.pose = new FilteredPose(() -> camera.getPoseEstimate(PoseEstimateType.TARGET_POSE_ROBOT_SPACE).getPose())
//                     .addMedianFilter(50);
//         this.xController = new ProfiledPIDController(0.01, 0, 0, new Constraints(1, 1));
//         this.yController = new ProfiledPIDController(0.01, 0, 0, new Constraints(1, 1));
//         this.thetaController = new ProfiledPIDController(0.01, 0, 0, new Constraints(1, 1));
//     }

//     @Override
//     public void initialize() {
        
//     }

//     @Override
//     public void execute() {
//         if(!camera.hasValidTarget()) return;
//         if (idFilter.isEmpty()) idFilter.add(camera.getAprilTagID());
//         if (!idFilter.contains(camera.getAprilTagID())) return;

//         Pose2d p = pose.getFiltered();

//         double x = xController.calculate(p.getX(), 0);
//         double y = yController.calculate(p.getY(), 0);
//         double theta = thetaController.calculate(p.getRotation().getRadians(), 0);

//         speeds.setFeedbackSpeeds(x, y, theta);
//     }

//     @Override
//     public void end(boolean interrupted) {
//         speeds.stopFeedbackSpeeds();
//     }

//     @Override
//     public boolean isFinished() {
//        return false;
//     }    
// }
