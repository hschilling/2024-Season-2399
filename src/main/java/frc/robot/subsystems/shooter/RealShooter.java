package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.utils.MotorUtil;

public class RealShooter implements ShooterIO {

    public static CANSparkMax shooterMotorControllerLow;
    public static CANSparkMax shooterMotorControllerHigh;
    public static RelativeEncoder shooterLowEncoder;
    public static RelativeEncoder shooterHighEncoder;
    public static SparkPIDController shooterHighController;
    public static SparkPIDController shooterLowController;
    private double slewRate = 0.2;

    public RealShooter()
    {
        shooterMotorControllerLow = MotorUtil.createSparkMAX(ShooterConstants.SHOOT_LOW_MOTOR_ID, MotorType.kBrushless, 
            Constants.NEO_CURRENT_LIMIT, true, true, slewRate);
        
        shooterMotorControllerHigh = MotorUtil.createSparkMAX(ShooterConstants.SHOOT_HIGH_MOTOR_ID, MotorType.kBrushless, 
            Constants.NEO_CURRENT_LIMIT, true, true, slewRate);

        // initialize motor encoder
        shooterLowEncoder = shooterMotorControllerLow.getEncoder();
        shooterHighEncoder = shooterMotorControllerHigh.getEncoder();
        shooterHighEncoder.setPositionConversionFactor(ShooterConstants.SHOOTER_POS_CONVERSION_FACTOR);
        shooterLowEncoder.setPositionConversionFactor(ShooterConstants.SHOOTER_POS_CONVERSION_FACTOR);
        shooterHighEncoder.setVelocityConversionFactor(ShooterConstants.SHOOTER_VEL_CONVERSION_FACTOR);
        shooterLowEncoder.setVelocityConversionFactor(ShooterConstants.SHOOTER_VEL_CONVERSION_FACTOR);

        //initialize PID controllers, set gains
        shooterHighController = shooterMotorControllerHigh.getPIDController();
        shooterLowController = shooterMotorControllerLow.getPIDController();
        
        shooterHighController.setFeedbackDevice(shooterHighEncoder);
        shooterLowController.setFeedbackDevice(shooterLowEncoder);
  
 
        SmartDashboard.putNumber("shooter FF", 0);
        SmartDashboard.putNumber("shooter P", 0);



    }

    //Basic shooting command
    @Override
    public void setMotor(double shootSpeed) {
        shooterMotorControllerLow.set(shootSpeed);
        shooterMotorControllerHigh.set(shootSpeed);
    }

    //Shooting command, but using PID
    @Override
    public void setSpeed(double speedPercent) {
        shooterHighController.setReference(speedPercent * ShooterConstants.SHOOTER_MAX_SPEED_MPS, ControlType.kVelocity);
        shooterLowController.setReference(speedPercent * ShooterConstants.SHOOTER_MAX_SPEED_MPS, ControlType.kVelocity);    
        SmartDashboard.putNumber("shooter reference", speedPercent * ShooterConstants.SHOOTER_MAX_SPEED_MPS);
        SmartDashboard.putNumber("shooter speed (RPM)", getEncoderSpeed());

    }

    public double getCurrent()
    {
        return shooterMotorControllerHigh.getOutputCurrent();
    }

    @Override
    public double getEncoderSpeed() {
        return shooterHighEncoder.getVelocity();
    }

    @Override
    public void setCurrentLimit(int current) {
        shooterMotorControllerHigh.setSmartCurrentLimit(current);        
    }

    @Override
    public void periodicUpdate() {
        // SmartDashboard.putNumber("intake/current (A)", getCurrent());
        // SmartDashboard.putNumber("intake/temp (C)", shooterMotorControllerHigh.getMotorTemperature());  
        shooterLowController.setFF(SmartDashboard.getNumber("shooter FF", 0));
        shooterHighController.setFF(SmartDashboard.getNumber("shooter FF", 0));
        shooterHighController.setP(SmartDashboard.getNumber("shooter P", 0));
        shooterLowController.setP(SmartDashboard.getNumber("shooter P", 0));      
    }

}
