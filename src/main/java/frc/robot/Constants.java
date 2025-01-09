package frc.robot;

import ca.frc6390.athena.commands.SwerveDriveCommand.SwerveDriveCommandConfig;
import ca.frc6390.athena.drivetrains.swerve.SwerveHelpers;
import ca.frc6390.athena.drivetrains.swerve.SwerveModule.SwerveEncoder;
import ca.frc6390.athena.drivetrains.swerve.SwerveModule.SwerveModuleConfig;
import ca.frc6390.athena.drivetrains.swerve.SwerveModule.SwerveMotor;
import ca.frc6390.athena.drivetrains.swerve.modules.SDSModules;
import ca.frc6390.athena.drivetrains.swerve.modules.SDSModules.MK4i;
import ca.frc6390.athena.drivetrains.swerve.modules.SDSModules.SDSMotor;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public interface Constants {
    
    public interface DriveTrain {
        double TRACKWIDTH_METERS = Units.inchesToMeters(24);
        double WHEELBASE_METERS = Units.inchesToMeters(24);

        String CANBUS = "can";
        SDSModules MODULE = new SDSModules(SDSMotor.KRAKEN_X60, MK4i.L1, CANBUS);
        PIDController ROTATION_PID = new PIDController(0, 0, 0);
        PIDController DRIFT_PID = new PIDController(0, 0, 0);

        //LF,FR,BL,BR
        int[] DRIVE_IDS = {5,14,6,17};
        int[] ROTATION_IDS = {15,12,8,9};
        int[] ENCODER_IDS = {1,2,3,4};
        double[] ENCODER_OFFSETS = {0,0,0,0};
        int PIGEON_ID = 20;

        Translation2d[] MODULE_LOCATIONS = SwerveHelpers.generateModuleLocations(TRACKWIDTH_METERS, WHEELBASE_METERS);
        SwerveMotor[] DRIVE_MOTORS = MODULE.generateDriveMotors(DRIVE_IDS);
        SwerveMotor[] ROTATION_MOTORS = MODULE.generateRotationMotors(ROTATION_IDS);
        SwerveEncoder[] MODULE_ENCODERS = MODULE.generateEncoders(ENCODER_IDS, ENCODER_OFFSETS);
        SwerveModuleConfig[] MODULE_CONFIGS = MODULE.generateConfigs(MODULE_LOCATIONS, DRIVE_MOTORS, ROTATION_MOTORS, ROTATION_PID, MODULE_ENCODERS);

        double THETA_DEADZONE = 0.15;
        SwerveDriveCommandConfig DRIVE_COMMAND_CONFIG = new SwerveDriveCommandConfig(THETA_DEADZONE, MODULE.calculateFreeSpeedMeter(), MODULE.calculateFreeSpeedMeter());
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
