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

    table.add(new LookUpTableEntry(1.61, 56));
    table.add(new LookUpTableEntry(1.738, 55));
    table.add(new LookUpTableEntry(1.8743, 53));
    table.add(new LookUpTableEntry(2.00586, 48));
    table.add(new LookUpTableEntry(2.1455, 46));
    table.add(new LookUpTableEntry(2.2937, 42));
    table.add(new LookUpTableEntry(2.44089, 40));
    table.add(new LookUpTableEntry(2.5780, 39));

    table.add(new LookUpTableEntry(2.7423408, 35));
    table.add(new LookUpTableEntry(2.801, 34));

    table.add(new LookUpTableEntry(2.84599, 34));
    table.add(new LookUpTableEntry(2.880, 32));
    table.add(new LookUpTableEntry(2.9976, 30));

    table.add(new LookUpTableEntry(3.165, 30));
    table.add(new LookUpTableEntry(3.3277, 28));
    table.add(new LookUpTableEntry(3.48137, 26));
    table.add(new LookUpTableEntry(3.68, 26));
    table.add(new LookUpTableEntry(3.877, 26));
    table.add(new LookUpTableEntry(3.967, 24));
    table.add(new LookUpTableEntry(4.12278, 23));
    table.add(new LookUpTableEntry(4.3035, 23));
    table.add(new LookUpTableEntry(4.565, 23));
    table.add(new LookUpTableEntry(4.8058, 22));
    table.add(new LookUpTableEntry(5.0639, 21));
    table.add(new LookUpTableEntry(5.3029, 20.5));

    // 6200 rpm

    // table.add(new LookUpTableEntry(1.831,54));
    // table.add(new LookUpTableEntry(2.117,48));
    // table.add(new LookUpTableEntry(2.46,44));
    // table.add(new LookUpTableEntry(2.88537,32));
    // table.add(new LookUpTableEntry(1.837,51));
    // table.add(new LookUpTableEntry(2.01,47));
    // table.add(new LookUpTableEntry(2.3138,44));
    // table.add(new LookUpTableEntry(2.503,40));

    // table.add(new LookUpTableEntry(2.646,38));
    // table.add(new LookUpTableEntry(2.705,37));
    // table.add(new LookUpTableEntry(2.906,37));
    // table.add(new LookUpTableEntry(3.144,32));
    // table.add(new LookUpTableEntry(3.355,29));
    // table.add(new LookUpTableEntry(3.542,29));
    // table.add(new LookUpTableEntry(3.9427,24));
    // table.add(new LookUpTableEntry(4.339,22));
    // table.add(new LookUpTableEntry(4.226,25));
    // table.add(new LookUpTableEntry(5.082369,22.2));
    // table.add(new LookUpTableEntry(5.76322,21));

  }

  static {
    fillInTable();
  }

  public static LookUpTableEntry calcShooterTableEntry(double distance) {
    LookUpTableEntry closestLower = table.get(0);
    LookUpTableEntry closestHigher = table.get(table.size() - 1);

    if (distance <= closestLower.distance) return closestLower;
    if (distance >= closestHigher.distance && distance < closestHigher.distance + 1)
      return closestHigher;
    // return new LookUpTableEntry(distance, 56);
    if (distance >= closestHigher.distance + 1) return new LookUpTableEntry(distance, 56);

    for (LookUpTableEntry entry : table) {
      if (entry.distance < distance
          && (Math.abs(distance - closestLower.distance) > Math.abs(distance - entry.distance))) {
        closestLower = entry;
      } else if (entry.distance > distance
          && (Math.abs(closestHigher.distance - distance) > Math.abs(entry.distance - distance))) {
        closestHigher = entry;
      } else if (entry.distance == distance) {
        return entry;
      }
    }

    double scaleFactor =
        (distance - closestLower.distance) / (closestHigher.distance - closestLower.distance);

    double calculatedAngle =
        scaleFactor * (closestLower.angle - closestHigher.angle) + closestHigher.angle;

    return new LookUpTableEntry(distance, calculatedAngle);
  }
}
