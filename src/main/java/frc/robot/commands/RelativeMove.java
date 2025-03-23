package frc.robot.commands;

import ca.frc6390.athena.core.RobotBase;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

public class RelativeMove extends Command{

    RobotBase<?> base;
    double x,y,z;

    public RelativeMove(RobotBase<?> base, double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;
        this.base = base;
    }

    @Override
    public void initialize() {
        base.getLocalization().resetRelativePose(0, 0,0);
    }

    @Override
    public void execute() {
        base.getRobotSpeeds().setSpeeds("feedback", ChassisSpeeds.fromFieldRelativeSpeeds(x, y, z, base.getLocalization().getRelativePose().getRotation()));
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
