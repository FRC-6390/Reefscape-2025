package frc.robot;

import com.pathplanner.lib.config.PIDConstants;

import ca.frc6390.athena.core.RobotBase.RobotBaseConfig;
import ca.frc6390.athena.core.RobotDrivetrain.RobotDriveTrainIDs.DrivetrainIDs;
import ca.frc6390.athena.core.RobotLocalization.RobotLocalizationConfig;
import ca.frc6390.athena.core.RobotVision.RobotVisionConfig;
import ca.frc6390.athena.devices.Encoder.EncoderType;
import ca.frc6390.athena.devices.IMU.IMUType;
import ca.frc6390.athena.devices.MotorController.Motor;
import ca.frc6390.athena.drivetrains.swerve.SwerveDrivetrain;
import ca.frc6390.athena.drivetrains.swerve.SwerveDrivetrain.SwerveDrivetrainConfig;
import ca.frc6390.athena.drivetrains.swerve.modules.SwerveVendorSDS;
import ca.frc6390.athena.mechanisms.Mechanism;
import ca.frc6390.athena.mechanisms.Mechanism.MechanismConfig;
import ca.frc6390.athena.mechanisms.TurretMechanism.StatefulTurretMechanism;
import ca.frc6390.athena.sensors.camera.limelight.LimelightConfig;
import ca.frc6390.athena.sensors.camera.limelight.LimeLight.PoseEstimateWithLatencyType;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.superstructure.EndEffector.EndEffectorState;

public interface Constants {
    
    public interface DriveTrain {
        double TRACKWIDTH_METERS = Units.inchesToMeters(18.375); 

        String CANBUS = "can";

        // SAM OFFSETS
        // double[] ENCODER_OFFSETS = {0.697,3.123,2.445,-0.851};

        // OLD PATRICK OFFSETS
        double[] ENCODER_OFFSETS = {0.366943359375, 0.7001953125,0.176513671875,0.38476562499999994};

        // NEW PATRICK OFFSETS
        // double[] ENCODER_OFFSETS = {0,0,0,0};
        
        //OTHER WAY
        // double[] ENCODER_OFFSETS = {-0.86669921875, -0.19970703125,-0.67333984375,-0.880126953125};
        
        //REQUIRES INVERSION OF TWO MOTORS
        // double[] ENCODER_OFFSETS = {-0.366943359375, -0.19970703125,-0.176513671875,-0.880126953125};
        
        SwerveDrivetrainConfig DRIVETRAIN_CONFIG = new SwerveDrivetrainConfig(IMUType.CTREPigeon2, false)
                                                    .sameModule(SwerveVendorSDS.MK4i.L3.config(Motor.KRAKEN_X60,EncoderType.CTRECANcoder))
                                                    .setModuleLocations(TRACKWIDTH_METERS)
                                                    .setRotationPID(new PIDController(0.5 , 0,0))
                                                    .setIDs(DrivetrainIDs.SWERVE_CHASSIS_STANDARD)
                                                    .setOffsets(ENCODER_OFFSETS)
                                                    .setCanbus(CANBUS)
                                                    .setDriveInverted(false)
                                                    .setFieldRelative(false)
                                                    .setDriftCorrectionPID(new PIDController(0, 0, 0))
                                                    
                                                    .setDriftActivationSpeed(0.0);

        RobotLocalizationConfig LOCALIZATION_CONFIG = new RobotLocalizationConfig().setPoseEstimateOrigin(PoseEstimateWithLatencyType.BOT_POSE_MT2_BLUE).setVision(0.3, 0.3, 9999).setAutoPlannerPID(new PIDConstants(5,0,0), new PIDConstants(2,0,0)).setVisionEnabled(true);

        RobotBaseConfig<SwerveDrivetrain> ROBOT_BASE = RobotBaseConfig.swerve(DRIVETRAIN_CONFIG)
                                                                      .setLocalization(LOCALIZATION_CONFIG)
                                                                      .setVision(RobotVisionConfig.limelight(new LimelightConfig("limelight-driver").setAngleRelativeToForwards(180),new LimelightConfig("limelight-tag").setAngleRelativeToForwards(0)));
    }

    public interface Controllers {
        double STICK_DEADZONE = 0.15;      
    }
    public interface Climber {
        int LIMIT_SWITCH = 1;
        int ENCODER = 40;
        int LEFT_MOTOR = 31;
        int RIGHT_MOTOR = 30;
        double ENCODER_OFFSET = 0;
        double ENCODER_GEAR_RATIO = 4d/1d;
        String CANBUS = "can";
        PIDController CONTORLLER = new PIDController(0.015, 0, 0);


        // MechanismConfig<StatefulMechanism<ClimberState>> CLIMBER_CONFIG = MechanismConfig.statefulGeneric(ClimberState.Home)
        //                                                                                 .addMotors(Motor.KRAKEN_X60, 30,-31)
        //                                                                                 .setEncoder(EncoderType.CTRECANcoder, 40)
        //                                                                                 .setEncoderGearRatio(4d/1d)
        //                                                                                 .setEncoderConversion(360)
        //                                                                                 .setEncoderOffset(0)
        //                                                                                 .setCanbus(CANBUS)
        //                                                                                 .setPID(0.015, 0, 0)
        //                                                                                 .addLowerLimitSwitch(0, 0, true);

    }

    public interface Elevator {
        String CANBUS = "rio";
        int ENCODER = 23;
        int LEFT_MOTOR = 20;
        int RIGHT_MOTOR = 21;
        double GEAR_DIAMETER_INCHES = 2d;
        double OFFSET_FROM_FLOOR = 0;
        double ENCODER_GEAR_RATIO = 1d/1d;
        double MOTOR_GEAR_RATIO = 6d/1d;
        int LIMIT_SWITCH = 0;
        //MAX ACCEL WAS 15
        ProfiledPIDController CONTORLLER = new ProfiledPIDController(0.11, 0.01, 0, new Constraints(60, 18));
        ElevatorFeedforward FEEDFORWARD = new ElevatorFeedforward(0, 0.17, 0.377,0.78);
        
        // ProfiledPIDController CONTORLLER = new ProfiledPIDController(0.11, 0, 0, new Constraints(60, 30));
        // ElevatorFeedforward FEEDFORWARD = new ElevatorFeedforward(0, 0.129, 0.377,0.75);
        // ProfiledPIDController CONTORLLER = new ProfiledPIDController(0.2, 0, 0, new Constraints(0.5, 0.5));
        // ElevatorFeedforward FEEDFORWARD = new ElevatorFeedforward(0, 0.12875, 0.688,0.00);

        // ProfiledPIDController CONTORLLER = new ProfiledPIDController(0.11, 0.0095, 0, new Constraints(0.5, 0), 5);
        // ElevatorFeedforward FEEDFORWARD = new ElevatorFeedforward(0, 0.04, 2.67,0.006666667);

        // MechanismConfig<StatefulElevatorMechanism<ElevatorState>> ELEVATOR_CONFIG = MechanismConfig.statefulElevator(new ElevatorFeedforward(0, 0.17, 0.377,0.78), ElevatorState.StartConfiguration)
        //                                                                             .addMotors(Motor.KRAKEN_X60, 20,21)
        //                                                                             .setEncoder(EncoderType.CTRECANcoder, 23)
        //                                                                             .setCanbus(CANBUS)
        //                                                                             .setEncoderGearRatio(1d/1d)
        //                                                                             .setEncoderConversion(2d)
        //                                                                             .setProfiledPID(0.11, 0.01, 0, new Constraints(60, 18))
        //                                                                             .addLowerLimitSwitch(0, 0, true);
    }

    public interface EndEffector {
        int LIMIT_SWITCH = 1;
        int ENCODER = 40;
        int MOTOR = 31;
        // int RIGHT_MOTOR = 30;
        double ENCODER_OFFSET = 0;
        double ENCODER_GEAR_RATIO = 1d/1d; //from motors 125d/1d;
        String CANBUS = "rio";
        PIDController CONTORLLER = new PIDController(0.015, 0, 0);

        MechanismConfig<StatefulTurretMechanism<EndEffectorState>> ENDEFFECTOR_CONFIG = MechanismConfig.statefulTurret(new SimpleMotorFeedforward(0, 0,0,0), EndEffectorState.StartConfiguration)
                                                                                    .addMotor(Motor.KRAKEN_X60, 31)
                                                                                    .setEncoder(EncoderType.CTRECANcoder, 40)
                                                                                    .setCanbus(CANBUS)
                                                                                    .setEncoderGearRatio(1d/1d)
                                                                                    .setEncoderConversion(360d)
                                                                                    .setEncoderOffset(0)
                                                                                    .setUseEncoderAbsolute(true)
                                                                                    .setProfiledPID(0,0,0, new Constraints(0, 0));

        MechanismConfig<Mechanism> SCORER_CONFIG = MechanismConfig.generic()
                                                                    .addMotor(Motor.KRAKEN_X60, 32)
                                                                    .setCanbus(CANBUS);


        // MechanismConfig<Mechanism> ALGEA_CONFIG = MechanismConfig.generic()
        //                                                             .addMotor(Motor.KRAKEN_X60, 33)
        //                                                             .setCanbus(CANBUS)
        //                                                             .setEncoderFromMotor(32)
        //                                                             .addUpperLimitSwitch(2, 0, true);      
                                                                                                                                          
    }
}
