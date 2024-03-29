// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.util;

import java.util.ArrayList;


/** Add your docs here. */
public class LookUpTable {

    public static final class LookUpTableEntry {
        public double distance;
        public double angle;

        public LookUpTableEntry(double distance, double angle) {
            this.distance = distance;
            this.angle = angle;

        }

        
    }

    private static ArrayList<LookUpTableEntry> table = new ArrayList<>();

    private static void fillInTable() {
        table.add(new LookUpTableEntry(0,56));

    }
    static {
        fillInTable();
    }


    public static LookUpTableEntry calcShooterTableEntry(double distance) {
        LookUpTableEntry closestLower = table.get(0);
        LookUpTableEntry closestHigher = table.get(table.size() - 1);

        if (distance <= closestLower.distance)
            return closestLower;
        if (distance >= closestHigher.distance)
            return closestHigher;
        for(LookUpTableEntry entry : table) {
            if (entry.distance < distance
                    && (Math.abs(distance - closestLower.distance) > Math
                            .abs(distance - entry.distance))) {
                closestLower = entry;
            } else if (entry.distance > distance
                    && (Math.abs(closestHigher.distance - distance) > Math
                            .abs(entry.distance - distance))) {
                closestHigher = entry;
            } else if (entry.distance == distance) {
                return entry;
            }


        }

        double scaleFactor = (distance -closestLower.distance)
                / (closestHigher.distance -closestLower.distance);

       

        double calculatedAngle = scaleFactor * (closestLower.angle - closestHigher.angle) + closestHigher.angle;

        return new LookUpTableEntry(distance, calculatedAngle);
    }
}
