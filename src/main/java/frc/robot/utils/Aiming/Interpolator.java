// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.Aiming;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.Pair;

/** Add your docs here. */
public class Interpolator 
{
    public static double interpolate(Map<Double, Double> ogMap, double x) {
    
    HashMap<Double, Double> map = new HashMap<Double,Double>(ogMap);
    if (map == null || map.isEmpty()) return 0;

    
    List<Double> keys = new ArrayList<>(map.keySet());
    Collections.sort(keys);

    if (x <= keys.get(0)) return map.get(keys.get(0));
    if (x >= keys.get(keys.size() - 1)) return map.get(keys.get(keys.size() - 1));

    double lowerKey = keys.get(0);
    double upperKey = keys.get(keys.size() - 1);

    for (int i = 0; i < keys.size() - 1; i++) {
        double k1 = keys.get(i);
        double k2 = keys.get(i + 1);

        if (x >= k1 && x <= k2) {
            lowerKey = k1;
            upperKey = k2;
            break;
        }
    }

    double y1 = map.get(lowerKey);
    double y2 = map.get(upperKey);

    return y1 + (x - lowerKey) * ( (y2 - y1) / (upperKey - lowerKey) );
}
}
