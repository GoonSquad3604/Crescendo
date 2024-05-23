package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;


public class Shooter extends SubsystemBase {

    private static Shooter _instance;
    private CANSparkMax turretMotor;
    private TalonFX shooterMotor;

    


    public static Shooter getInstance() {
        if (_instance == null) {
            _instance = new Shooter();
        }
        return _instance;
    }
    

    public Shooter() {
        turretMotor = new CANSparkMax(7, MotorType.kBrushless);
        turretMotor.restoreFactoryDefaults();

        shooterMotor = new TalonFX(8);

       // in init function, set slot 0 gains
var shooterConfigs = new Slot0Configs();
shooterConfigs.kS = 0.05; // Add 0.05 V output to overcome static friction
shooterConfigs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
shooterConfigs.kP = 0.11; // An error of 1 rps results in 0.11 V output
shooterConfigs.kI = 0; // no output for integrated error
shooterConfigs.kD = 0; // no output for error derivative

shooterMotor.getConfigurator().apply(shooterConfigs);

        // create a velocity closed-loop request, voltage output, slot 0 configs
// final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);

// // set velocity to 8 rps, add 0.5 V to overcome gravity
// shooterMotor.setControl(m_request.withVelocity((1.0/60)).withFeedForward(0.5));
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
        // shooterMotor.set(-.6);
        final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);

// set velocity to 8 rps, add 0.5 V to overcome gravity
shooterMotor.setControl(m_request.withVelocity((-85)).withFeedForward(0.5));
    }

    public void stopShooter(){
        shooterMotor.set(0);
    }

    public void runShooterReverse(){
        shooterMotor.set(.225);
    }

    public void setPower(double RPM) {
        
    }
}
