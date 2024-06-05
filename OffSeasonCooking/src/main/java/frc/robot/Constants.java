package frc.robot;

public final class Constants {

  public static final class General {
    public static final double deadband = 0.1;

    /*Robot speed is the max speed the robot will run at.
      speed of 1 means 100%, and 1.1 would mean 110%*/
    public static final double robotSpeed = 1.0;
    public static final int pigeonID = 12;
  }

  public static final class DriveConstants {
    public static final int leftFrontID = 1;
    public static final int rightFrontID = 2;
    public static final int leftRearID = 3;
    public static final int rightRearID = 4;
  }

  public static final class IndexerConstants {
    public static final int indexer1ID = 5;
    public static final int indexer2ID = 6;
  }

  public static final class ShooterConstants {
    public static final int turretID = 7;
    public static final int shooterID = 8;
  }

  public static final class IntakeConstants {
    public static final int spikeWheelID = 9;
    public static final int hingeID = 10;
    public static final int solidWheelID = 11;
  }
}
