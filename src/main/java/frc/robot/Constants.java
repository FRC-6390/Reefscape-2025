package frc.robot;

import ca.frc6390.athena.commands.SwerveDriveCommand.SwerveDriveCommandConfig;
import ca.frc6390.athena.drivetrains.swerve.SwerveHelpers;
import ca.frc6390.athena.drivetrains.swerve.SwerveModule.SwerveEncoder;
import ca.frc6390.athena.drivetrains.swerve.SwerveModule.SwerveModuleConfig;
import ca.frc6390.athena.drivetrains.swerve.SwerveModule.SwerveMotor;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public interface Constants {
    
    public interface DriveTrain {
        
        int PIDGEON_ID = 20;
        double SWERVE_WHEEL_DIAMETER_METERS = Units.inchesToMeters(4);

        double TRACKWIDTH_METERS = Units.inchesToMeters(24);
        double WHEELBASE_METERS = Units.inchesToMeters(24);

        String SWERVE_CAN_BUS = "can";
        PIDController ROTATION_PID = new PIDController(0, 0, 0);
        PIDController DRIFT_PID = new PIDController(0, 0, 0);

        //LF,FR,BL,BR
        int[] DRIVE_IDS = {5,14,6,17};
        int[] ROTATION_IDS = {15,12,8,9};
        int[] ENCODER_IDS = {1,2,3,4};
        double[] ENCODER_OFFSETS = {0,0,0,0};

        

        Translation2d[] MODULE_LOCATIONS = SwerveHelpers.generateModuleLocations(TRACKWIDTH_METERS, WHEELBASE_METERS);
        SwerveMotor[] DRIVE_MOTORS = SwerveHelpers.generateMotors((8.14d/1d), Units.feetToMeters(12.9), SWERVE_CAN_BUS, DRIVE_IDS);
        SwerveMotor[] ROTATION_MOTORS = SwerveHelpers.generateMotors((150d/7d)/1d, Units.feetToMeters(12.9), SWERVE_CAN_BUS, ROTATION_IDS);
        SwerveEncoder[] MODULE_ENCODERS = SwerveHelpers.generateEncoders(1, ENCODER_OFFSETS, SWERVE_CAN_BUS, ENCODER_IDS);

        SwerveModuleConfig[] MODULE_CONFIGS = SwerveHelpers.generateConfigs(MODULE_LOCATIONS, SWERVE_WHEEL_DIAMETER_METERS, DRIVE_MOTORS, ROTATION_MOTORS, ROTATION_PID, MODULE_ENCODERS);


        double MAX_SPEED_METERS_PER_SECOND = Units.feetToMeters(13.5);
        double MAX_ANGULAR_SPEED_METERS_PER_SECOND = Units.feetToMeters(13.5);
        double THETA_DEADZONE = 0.15;

        SwerveDriveCommandConfig DRIVE_COMMAND_CONFIG = new SwerveDriveCommandConfig(THETA_DEADZONE, MAX_SPEED_METERS_PER_SECOND, MAX_ANGULAR_SPEED_METERS_PER_SECOND);
    }

    public interface Elevator {
    
        enum Position {
            L1(-1),
            L2(-1),
            L3(-1),
            L4(-1),
            Feeder(-1);

            double pos;
            private Position(double pos){
                this.pos = pos;
            }

            public double getPosition() {
                return pos;
            }
        }
    }

    public interface FieldElements {

        enum ReefAprilTag {
            Red(-1),
            Blue(-1);
            double pos;
            private ReefAprilTag(double pos){
                this.pos = pos;
            }

            public double getPosition() {
                return pos;
            }
        }

    }
}
