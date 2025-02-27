package frc.robot;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;

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
import ca.frc6390.athena.sensors.camera.limelight.LimeLightConfig;
import ca.frc6390.athena.sensors.camera.limelight.LimeLight.PoseEstimateWithLatencyType;
import ca.frc6390.athena.sensors.camera.photonvision.PhotonVisionConfig;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.superstructure.EndEffector.EndEffectorState;
import frc.robot.utils.ReefScoringPos.ReefPole;

public interface Constants {
    
    public interface DriveTrain {
        double TRACKWIDTH_METERS = Units.inchesToMeters(18.375); 

        String CANBUS = "can";

        // SAM OFFSETS
        // double[] ENCODER_OFFSETS = {0.697,3.123,2.445,-0.851};

        // OLD PATRICK OFFSETS
        double[] ENCODER_OFFSETS = {0.7294921875, 0.599609375,0.6860351562500001, 0.8649902343749999};

        // NEW PATRICK OFFSETS
        // double[] ENCODER_OFFSETS = {0,0,0,0};
        
        //OTHER WAY
        // double[] ENCODER_OFFSETS = {-0.86669921875, -0.19970703125,-0.67333984375,-0.880126953125};
        
        //REQUIRES INVERSION OF TWO MOTORS
        // double[] ENCODER_OFFSETS = {-0.366943359375, -0.19970703125,-0.176513671875,-0.880126953125};
        
        SwerveDrivetrainConfig DRIVETRAIN_CONFIG = new SwerveDrivetrainConfig(IMUType.CTREPigeon2, false)
                                                    .custom(
                                                            SwerveVendorSDS.MK4n.L1_PLUS.config(Motor.KRAKEN_X60, EncoderType.CTRECANcoder).setPID(new PIDController(0.45,0,0)), 
                                                            SwerveVendorSDS.MK4n.L1_PLUS.config(Motor.KRAKEN_X60,EncoderType.CTRECANcoder).setPID(new PIDController(0.45, 0, 0)), 
                                                            SwerveVendorSDS.MK4i.L1_PLUS.config(Motor.KRAKEN_X60,EncoderType.CTRECANcoder).setPID(new PIDController(0.5 , 0,0)), 
                                                            SwerveVendorSDS.MK4i.L1_PLUS.config(Motor.KRAKEN_X60,EncoderType.CTRECANcoder).setPID(new PIDController(0.5 , 0,0))
                                                            )   
                                                    .setModuleLocations(TRACKWIDTH_METERS)
                                                    .setIDs(DrivetrainIDs.SWERVE_CHASSIS_STANDARD)
                                                    .setOffsets(ENCODER_OFFSETS)
                                                    .setCanbus(CANBUS)
                                                    .setDriveInverted(false)
                                                    .setFieldRelative(false)
                                                    .setDriftCorrectionPID(new PIDController(0, 0, 0))
                                                    
                                                    .setDriftActivationSpeed(0.0)
                                                    .setCurrentLimit(60);

        RobotLocalizationConfig LOCALIZATION_CONFIG = new RobotLocalizationConfig().setSlipThresh(0.2).setVision(0.2, 0.2, 9999).setAutoPlannerPID(new PIDConstants(5,0,0), new PIDConstants(2,0,0)).setVisionEnabled(true);

        RobotBaseConfig<SwerveDrivetrain> ROBOT_BASE = RobotBaseConfig.swerve(DRIVETRAIN_CONFIG)
                                                                      .setLocalization(LOCALIZATION_CONFIG)
                                                                    //   .setVision(RobotVisionConfig.blank().addLimeLights(new LimeLightConfig("limelight-left").setYawRelativeToForwards(-90)));
                                                                    // .setVision(RobotVisionConfig.blank().addPhotonVision(new PhotonVisionConfig("OV9281", new Transform3d(-0.23495,0.9017,-0.1778, new Rotation3d()),PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR)).addLimeLights(new LimeLightConfig("limelight-left").setYawRelativeToForwards(-90),new LimeLightConfig("limelight-right").setYawRelativeToForwards(90)));

                                                                      .setVision(RobotVisionConfig.blank().addLimeLights(new LimeLightConfig("limelight-left").setYawRelativeToForwards(-90),new LimeLightConfig("limelight-right").setYawRelativeToForwards(90)));
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
        String CANBUS = "can";
        int ENCODER = 23;

        int LEFT_MOTOR = 20;
        int RIGHT_MOTOR = 21;
        double GEAR_DIAMETER_INCHES = 3d;
        double OFFSET_FROM_FLOOR = 24;
        double ENCODER_GEAR_RATIO = 1d/1d;
        double MOTOR_GEAR_RATIO = 6d/1d;
        int LIMIT_SWITCH = 5;
        //MAX ACCEL WAS 15
        ProfiledPIDController CONTORLLER = new ProfiledPIDController(0.06, 0.01, 0, new Constraints(50, 10));
        ElevatorFeedforward FEEDFORWARD = new ElevatorFeedforward(0, 0.208, 0.02,0.0);
        
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
        int LIMIT_SWITCH = 3;
        int ENCODER = 40;
        

        int MOTOR = 31;
        int ALGAE_MOTOR = 32;
        
        int ROLLER = 33;
        // double ENCODER_OFFSET = 0.551025390625;
        double ENCODER_OFFSET = -0.044921875;
        double ENCODER_GEAR_RATIO = 1d/1d; //from motors 125d/1d;
        String CANBUS = "can";
        PIDController CONTORLLER = new PIDController(0.015, 0, 0);
        // PIDController CONTORLLER = new PIDController(0.005, 0, 0);

        int CANDLE_ID = 22;

        // MechanismConfig<StatefulTurretMechanism<EndEffectorState>> ENDEFFECTOR_CONFIG = MechanismConfig.statefulTurret(new SimpleMotorFeedforward(0, 0,0,0), EndEffectorState.StartConfiguration)
        //                                                                             .addMotor(Motor.KRAKEN_X60, 31)
        //                                                                             .setEncoder(EncoderType.CTRECANcoder, 40)
        //                                                                             .setCanbus(CANBUS)
        //                                                                             .setEncoderGearRatio(1d/1d)
        //                                                                             .setEncoderConversion(360d)
        //                                                                             .setEncoderOffset(0)
        //                                                                             .setUseEncoderAbsolute(true)
        //                                                                             .setProfiledPID(0,0,0, new Constraints(0, 0));

        // MechanismConfig<Mechanism> SCORER_CONFIG = MechanismConfig.generic()
                                                                    // .addMotor(Motor.KRAKEN_X60, 32)
                                                                    // .setCanbus(CANBUS);


        // MechanismConfig<Mechanism> ALGEA_CONFIG = MechanismConfig.generic()
        //                                                             .addMotor(Motor.KRAKEN_X60, 33)
        //                                                             .setCanbus(CANBUS)
        //                                                             .setEncoderFromMotor(32)
        //                                                             .addUpperLimitSwitch(2, 0, true);      
                                                                                                                                          
    }
}
