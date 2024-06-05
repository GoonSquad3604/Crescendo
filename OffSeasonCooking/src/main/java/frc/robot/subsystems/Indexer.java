package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants;

public class Indexer extends SubsystemBase {
    
    private static Indexer _instance;
    private CANSparkMax indexMotor1;
    private CANSparkMax indexMotor2;
       

    public static Indexer getInstance() {
        if (_instance == null) {
            _instance = new Indexer();
        }
        return _instance;
    }
    

    public Indexer() {
        indexMotor1 = new CANSparkMax(Constants.IndexerConstants.indexer1ID, MotorType.kBrushless);
        indexMotor1.restoreFactoryDefaults();

        indexMotor2 = new CANSparkMax(Constants.IndexerConstants.indexer2ID, MotorType.kBrushless);
        indexMotor1.restoreFactoryDefaults();
    }

    public void runIndexer(){
        indexMotor1.set(-.3);
        indexMotor2.set(.3);
    }

    public void stopIndexer(){
        indexMotor1.set(0);
        indexMotor2.set(0);
    }

    public void runIndexerReverse(){
        indexMotor1.set(.3);
        indexMotor2.set(-.3);
    }
}
