// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.Aiming;

import java.util.ArrayList;

/** Add your docs here. */
public class ShotSolver 
{
    public static double rps_To_mps(double rps, double wheelRadiusinMetres)
    {
        return rps * 2 * Math.PI * wheelRadiusinMetres;
    }

    public static double[] solveQuadratic(double a, double b, double c) {
    double[] roots = new double[2];

    if (a == 0) {
        roots[0] = Double.POSITIVE_INFINITY;
        roots[1] = Double.POSITIVE_INFINITY;
        return roots;
    }

    double discriminant = b * b - 4 * a * c;

    if (discriminant > 0) {
        double sqrtD = Math.sqrt(discriminant);
        roots[0] = (-b + sqrtD) / (2 * a);
        roots[1] = (-b - sqrtD) / (2 * a);
    } else if (discriminant == 0) {
        roots[0] = -b / (2 * a);
        roots[1] = Double.POSITIVE_INFINITY;
    } else {
        roots[0] = Double.POSITIVE_INFINITY;
        roots[1] = Double.POSITIVE_INFINITY;
    }

    return roots;
}


    public static double computeAngleInDegrees(
        double shooterHeight,
        double targetHeight,
        double horizontalDistToTarget,
        double shooterVelocityRPS,
        double shooterWheelRadius)
    {
        double v = rps_To_mps(shooterVelocityRPS, shooterWheelRadius);
        double x = horizontalDistToTarget;
        double y = targetHeight - shooterHeight;
        double g = 9.8;

        double a = (g * x * x) / (2 * v * v);
        double b = -x;
        double c = y + a;

        double[] roots = solveQuadratic(a, b, c);

        double bestTan = Double.NEGATIVE_INFINITY;

        for (double r : roots) {
            if (r != Double.POSITIVE_INFINITY) {
                bestTan = Math.max(bestTan, r);
            }
        }

        if (bestTan == Double.NEGATIVE_INFINITY) {
            return Double.POSITIVE_INFINITY;
        }

        return Math.toDegrees(Math.atan(bestTan));
    }

}
