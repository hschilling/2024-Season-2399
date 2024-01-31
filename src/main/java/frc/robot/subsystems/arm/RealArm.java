package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.utils.MotorUtil;

public class RealArm implements ArmIO {
    private static CANSparkMax armMotorControllerLeft;
    private static CANSparkMax armMotorControllerRight;
    public static RelativeEncoder armEncoderLeft;
    public static RelativeEncoder armEncoderRight;
    // public static DutyCycleEncoder armAbsoluteEncoder;

    public RealArm() {
        // armAbsoluteEncoder = new DutyCycleEncoder(0);
        //Higher slew rate of .75 seconds from 0 to 100% (sparkmax thinks we use this) translates to .2 seconds from 0 to 20% (what we actually use)
        //Make the arm motor controllers 
        armMotorControllerLeft = MotorUtil.createSparkMAX(7, MotorType.kBrushless, Constants.NEO_CURRENT_LIMIT, 
            false,  true, 0.75);
         armMotorControllerRight = MotorUtil.createSparkMAX(6, MotorType.kBrushless, Constants.NEO_CURRENT_LIMIT, 
            true, true, 0.75);
        //Make the arm encoders
        armEncoderLeft = armMotorControllerLeft.getEncoder();
        armEncoderRight = armMotorControllerRight.getEncoder();
        
        //Position and velocity conversion factors
        armEncoderLeft.setPositionConversionFactor(ArmConstants.RADIANS_PER_REVOLUTION);
        armEncoderLeft.setVelocityConversionFactor(ArmConstants.RADIANS_PER_REVOLUTION / 60);
        armEncoderRight.setPositionConversionFactor(ArmConstants.RADIANS_PER_REVOLUTION);
        armEncoderRight.setVelocityConversionFactor(ArmConstants.RADIANS_PER_REVOLUTION / 60);

        //sets the encoder to the correct initial value (the offset). does not actually move the arm. 
        armEncoderLeft.setPosition(ArmConstants.INITIAL_OFFSET);
        armEncoderRight.setPosition(ArmConstants.INITIAL_OFFSET);
    }

    // public double getAbsoluteEncoderPosition() {
    //     return -(armAbsoluteEncoder.getAbsolutePosition() - 0.88) * 2 * Math.PI / 3;
    // }

    @Override
    public void periodicUpdate() {
        SmartDashboard.putNumber("arm/temp (C)", armMotorControllerLeft.getMotorTemperature());
    }

    @Override
    public double getEncoderPosition() {
       //return getAbsoluteEncoderPosition();
        return armEncoderLeft.getPosition();
    }

    @Override
    public double getEncoderSpeed() {
        return armEncoderLeft.getVelocity();
    }

    @Override
    public void setSpeed(double speed) {
        armMotorControllerLeft.set(speed);
        armMotorControllerRight.set(speed);
    }

    @Override
    public void setPosition(double position) {
        armEncoderLeft.setPosition(position);
        armEncoderRight.setPosition(position);
    }

    @Override
    public double getArmCurrent() {
        return armMotorControllerLeft.getOutputCurrent();
    }
    
}
