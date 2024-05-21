package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkLowLevel.MotorType;  

public class Turret extends SubsystemBase {
    private static Turret _instance;
    private CANSparkMax turretMotor;
    private TalonFX shooterMotor;   

    public static Turret getInstance() {
        if (_instance == null) {
            _instance = new Turret();
        }
        return _instance;
    }
    

    public Turret() {
        turretMotor = new CANSparkMax(7, MotorType.kBrushless);
        turretMotor.restoreFactoryDefaults();

        shooterMotor = new TalonFX(8);
    }

    public void spinTurretClockwise(){
        turretMotor.set(.3);
    }


    public void spinTurretCounterClockwise(){
        turretMotor.set(-.3);
    }

    public void stopTurret(){
        turretMotor.set(0);
    }

    public void runShooter(){
        shooterMotor.set(-.6);
    }

    public void stopShooter(){
        shooterMotor.set(0);
    }

    public void runShooterReverse(){
        shooterMotor.set(.225);
    }
}
