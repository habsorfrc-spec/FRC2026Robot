
/*package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.ElevatorConstants.*;

public class ElevatorSubsystem extends SubsystemBase {

    private final TalonFX leader = new TalonFX(ELEVATOR_MOTOR_ID);
    private final SparkMax follower = new SparkMax(ELEVATOR_MOTOR_FOLLOWER_ID, MotorType.kBrushed);

    private final DutyCycleOut duty = new DutyCycleOut(0);

    public ElevatorSubsystem() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        leader.getConfigurator().apply(config);
        follower.


        // SparkMaxConfig followerConfig = new SparkMaxConfig();

        // followerConfig.idleMode(IdleMode.kBrake);
        // If it spins the wrong way:
        // followerConfig.inverted(true);

        // follower.configure(
        //         followerConfig,
        //         ResetMode.kResetSafeParameters,
        //         PersistMode.kPersistParameters);

        // Reset encoder
        leader.setPosition(0);

        // Brake mode (important for elevators)
        leader.setNeutralMode(NeutralModeValue.Brake);
    }

    public void elevateUp() {
        double pos = leader.getPosition().getValueAsDouble();

        if (pos < 30) { // upper limit
            leader.setControl(duty.withOutput(0.6));
            //follower.set(1);
        } else {
            stop();
        }
    }

    public void elevateDown() {
        double pos = leader.getPosition().getValueAsDouble();

        if (pos > -1) { // lower limit
            leader.setControl(duty.withOutput(-1));
            //follower.set(-1);
        } else {
            stop(); 
        }
    }

    @Override
    public void periodic() {
        double pos = leader.getPosition().getValueAsDouble();
        SmartDashboard.putNumber("Elev pos", pos);
    }

    public void stop() {
        leader.setControl(duty.withOutput(0));
        follower.stopMotor();
    }
}*/
//  קוד של עידן 
package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

// ספריות REVLib בגרסה החדשה ביותר (2025/2026)
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.ElevatorConstants.*;

public class ElevatorSubsystem extends SubsystemBase {

    // --- הגדרת בקרי המנועים ---
    private final TalonFX leader = new TalonFX(ELEVATOR_MOTOR_ID);
    
    // מנוע פחמים (CIM) מחובר ל-SparkMax
    private final SparkMax follower = new SparkMax(ELEVATOR_MOTOR_FOLLOWER_ID, MotorType.kBrushed);

    private final DutyCycleOut duty = new DutyCycleOut(0);

    public ElevatorSubsystem() {
        // ----------------------------------------
        // הגדרת המנוע הראשי (TalonFX - Phoenix 6)
        // ----------------------------------------
        TalonFXConfiguration talonConfig = new TalonFXConfiguration();
        leader.getConfigurator().apply(talonConfig);
        
        // איפוס אנקודר ל-0 בהפעלה
        leader.setPosition(0);
        
        // שינוי: המנוע הראשי הועבר למצב Coast (חופשי)
        leader.setNeutralMode(NeutralModeValue.Brake);


        // ----------------------------------------
        // הגדרת המנוע המשני (SparkMax - REVLib 2025/2026)
        // ----------------------------------------
        SparkMaxConfig sparkConfig = new SparkMaxConfig();
        
        // שינוי: המנוע המשני הועבר למצב Coast (חופשי)
        sparkConfig.idleMode(IdleMode.kCoast);
        
        // כיוון המנוע (משאיר על true כדי שיעבדו יחד)
        sparkConfig.inverted(true); 

        // החלת הגדרות וצריבה לזיכרון של הבקר
        follower.configure(
            sparkConfig,
            ResetMode.kResetSafeParameters, 
            PersistMode.kPersistParameters  
        );
    }

    public void elevateUp() {
        double pos = leader.getPosition().getValueAsDouble();

        // הגבול נשאר בדיוק אותו דבר (30)
        if (pos < 30) { 
            // הפעלת שני המנועים יחד כלפי מעלה ב-50% כוח
            leader.setControl(duty.withOutput(0.5));
            follower.set(0.5);
        } else {
            stop();
        }
    }

    public void elevateDown(int limit) {
        double pos = leader.getPosition().getValueAsDouble();

        // הגבול נשאר בדיוק אותו דבר (-5)
        if (pos > limit) { 
            // הפעלת שני המנועים יחד כלפי מטה ב-100%
            leader.setControl(duty.withOutput(-1.0));
            follower.set(-1.0);
        } else {
            stop(); 
        }
    }

    @Override
    public void periodic() {
        // מציג את המיקום ב-SmartDashboard
        double pos = leader.getPosition().getValueAsDouble();
        SmartDashboard.putNumber("Elev pos", pos);
    }

    public void stop() {
        // עצירת שני המנועים לחלוטין
        leader.setControl(duty.withOutput(0));
        follower.stopMotor();
    }
}