package frc.robot.subsystems.arm;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.utils.MotorUtil;

public class RealArm implements ArmIO {
    private static CANSparkMax armMotorControllerLeft;
    private static CANSparkMax armMotorControllerRight;
    public static AbsoluteEncoder armAbsoluteEncoderRight;

    private final SparkPIDController armPIDControllerRight;

    public RealArm() {
        // armAbsoluteEncoder = new DutyCycleEncoder(0);
        // Higher slew rate of .75 seconds from 0 to 100% (sparkmax thinks we use this)
        // translates to .2 seconds from 0 to 20% (what we actually use
        // armEncoderLeft.setPosition(ArmConstants.INITIAL_OFFSET);

        armMotorControllerRight = MotorUtil.createSparkMAX(ArmConstants.ARM_MOTOR_ID_RIGHT, MotorType.kBrushless,
                Constants.NEO_CURRENT_LIMIT,
                true, true, 0.75);
        
        armAbsoluteEncoderRight = armMotorControllerRight.getAbsoluteEncoder(Type.kDutyCycle);
        armPIDControllerRight = armMotorControllerRight.getPIDController();
        armPIDControllerRight.setFeedbackDevice(armAbsoluteEncoderRight);

        armAbsoluteEncoderRight.setPositionConversionFactor(ArmConstants.RADIANS_PER_REVOLUTION);
        armAbsoluteEncoderRight.setVelocityConversionFactor(ArmConstants.RADIANS_PER_REVOLUTION / 60);

        armMotorControllerLeft = MotorUtil.createSparkMAX(ArmConstants.ARM_MOTOR_ID_LEFT, MotorType.kBrushless,
            Constants.NEO_CURRENT_LIMIT,
            false, true, 0.75);
        //armMotorControllerLeft.follow(armMotorControllerRight);

        // armAbsoluteEncoderRight.setPosition(ArmConstants.INITIAL_OFFSET);
    }

    public double getAbsoluteEncoderPosition() {
        return -(armAbsoluteEncoderRight.getPosition() - 0.88) * 2 * Math.PI / 3;
    }

    @Override
    public void periodicUpdate() {
        SmartDashboard.putNumber("arm/temp (C)", armMotorControllerLeft.getMotorTemperature());
    }

    @Override
    public double getEncoderPosition() {
        // return getAbsoluteEncoderPosition();
        return armAbsoluteEncoderRight.getPosition();
    }

    @Override
    public double getEncoderSpeed() {
        return armAbsoluteEncoderRight.getVelocity();
    }

    @Override
    public void setSpeed(double speed) {
        armMotorControllerRight.set(speed);
        armMotorControllerLeft.set(speed);
    }

    @Override
    public void setPosition(double position) {
        armPIDControllerRight.setReference(position, ControlType.kPosition);
    }

    @Override
    public double getArmCurrent() {
        return armMotorControllerRight.getOutputCurrent();
    }

}
