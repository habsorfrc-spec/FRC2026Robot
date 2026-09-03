///  קוד של עידן 
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
        leader.setControl(duty.withOutput(1.0));
        follower.set(1.0);
    }

    public void elevateDown(int limit) {
            leader.setControl(duty.withOutput(-1.0));
            follower.set(-1.0);
    }

    @Override
    public void periodic() {
        // מציג את המיקום ב-SmartDashboard
        double pos = leader.getPosition().getValueAsDouble();
        SmartDashboard.putNumber("Elev pos", pos);
        SmartDashboard.putNumber("Elev 2 pos", follower.getOutputCurrent());
    }

    public void stop() {
        // עצירת שני המנועים לחלוטין
        leader.setControl(duty.withOutput(0));
        follower.stopMotor();
    }
}