package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;
import frc.utils.MotorUtil;
import edu.wpi.first.units.Units; //may need for conversions (rotations to inches)

public class Climber extends SubsystemBase {
    private CANSparkMax leftMotorController;
    private CANSparkMax rightMotorController;
    private RelativeEncoder leftEncoder, rightEncoder;
    private SparkPIDController leftPIDController, rightPIDController;

    // get info on slew rate and what motors are doing
    public static final GenericEntry slewRate = Shuffleboard.getTab("Params").addPersistent("Climber Slew Rate", 5.0)
            .getEntry();
    public static final GenericEntry leftClimberMotor = Shuffleboard.getTab("Driver")
            .addPersistent("Left Climber Motor", 0).getEntry();
    public static final GenericEntry rightClimberMotor = Shuffleboard.getTab("Driver")
            .addPersistent("Right Climber Motor", 0).getEntry();

    SlewRateLimiter filter;

    // private double climberSetpoint;

    public static final double CLIMBER_KP = 0;// 1.875;
    public static final double CLIMBER_KI = 0;// 0.006;
    public static final double CLIMBER_KD = 0;// 52.5;
    public static final double CLIMBER_KF = 0.000086; // 0.15;
    public static final double CLIMBER_KIZ = 0;
    public static final double CLIMBER_K_MAX_OUTPUT = 1;
    public static final double CLIMBER_K_MIN_OUTPUT = 0;
    public static final double CLIMBER_MAX_RPM = 5700;

    public Climber() {
        // initialize motor controllers
        leftMotorController = MotorUtil.createSparkMAX(ClimberConstants.LEFT_CLIMBER_MOTOR_ID, MotorType.kBrushless, 0,
                true, 0.0);
        rightMotorController = MotorUtil.createSparkMAX(ClimberConstants.RIGHT_CLIMBER_MOTOR_ID, MotorType.kBrushless,
                0, true, 0.0);

        // initialize motor encoder
        leftEncoder = leftMotorController.getEncoder();
        rightEncoder = rightMotorController.getEncoder();

        // initialize motor pid controllers
        leftPIDController = leftMotorController.getPIDController();
        rightPIDController = rightMotorController.getPIDController();

        // assigns values to PID controllers
        leftPIDController.setP(CLIMBER_KP);
        leftPIDController.setI(CLIMBER_KI);
        leftPIDController.setD(CLIMBER_KD);
        leftPIDController.setIZone(CLIMBER_KIZ);
        leftPIDController.setFF(CLIMBER_KF);
        leftPIDController.setOutputRange(CLIMBER_K_MIN_OUTPUT, CLIMBER_K_MAX_OUTPUT);

        rightPIDController.setP(CLIMBER_KP);
        rightPIDController.setI(CLIMBER_KI);
        rightPIDController.setD(CLIMBER_KD);
        rightPIDController.setIZone(CLIMBER_KIZ);
        rightPIDController.setFF(CLIMBER_KF);
        rightPIDController.setOutputRange(CLIMBER_K_MIN_OUTPUT, CLIMBER_K_MAX_OUTPUT);

        // invert the motor controllers so climber climbs right
        leftMotorController.setInverted(false);
        rightMotorController.setInverted(false);

        // reset encoders to zero
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);

        // get info on climber slew rate
        SmartDashboard.putNumber("Climber Slew Rate",
                SmartDashboard.getNumber("Climber Slew Rate", ClimberConstants.CLIMBER_SLEW));
        filter = new SlewRateLimiter(SmartDashboard.getNumber("Climber Slew Rate", ClimberConstants.CLIMBER_SLEW));
        System.out.println("Climber SlewRateLimiter "
                + SmartDashboard.getNumber("Climber Slew Rate", ClimberConstants.CLIMBER_SLEW));

    }

    @Override

    public void periodic() {
        // This method will be called once per scheduler run
        // SmartDashboard.putBoolean("Left Climber Extended", this.isLeftExtended());
        // SmartDashboard.putBoolean("Right Climber Extended", this.isRightExtended());
        SmartDashboard.putNumber("Left Climber Height", getLeftEncoderPosition());
        SmartDashboard.putNumber("Right Climber Hieght", getRightEncoderPosition());

    }

    // basic climbing command
    public void setLeftMotor(double speed) {
        if (isLeftRetracted()) {
            leftMotorController.set(0);
            leftClimberMotor.setDouble(0);
        } else {
            speed = filter.calculate(speed);
            leftMotorController.set(speed);
            leftClimberMotor.setDouble(speed);
        }

    }

    // climbing command using pid
    public void setLeftSpeed(double setpoint) {
        if (isLeftRetracted()) {
            leftMotorController.set(0);
            leftClimberMotor.setDouble(0);
        } else {

            leftPIDController.setReference(setpoint, CANSparkBase.ControlType.kPosition);
            // SmartDashboard.putNumber("Climber speed ", speed);
        }
    }

    // basic climbing command
    public void setRightMotor(double speed) {
        if (isRightRetracted()) {
            rightMotorController.set(0);
            rightClimberMotor.setDouble(0);
        } else {
            speed = filter.calculate(speed);
            rightMotorController.set(speed);
            rightClimberMotor.setDouble(speed);
        }

    }

    // climbing command using pid
    public void setRightSpeed(double setpoint) {
        if (isRightRetracted()) {
            rightMotorController.set(0);
            rightClimberMotor.setDouble(0);
        } else {

            rightPIDController.setReference(setpoint, CANSparkBase.ControlType.kPosition);
            // SmartDashboard.putNumber("Climber speed ", speed);
        }
    }

    public boolean isLeftExtended() {
        return (leftEncoder.getPosition() > ClimberConstants.MAX_HEIGHT);
    }

    public boolean isRightExtended() {
        return (rightEncoder.getPosition() > ClimberConstants.MAX_HEIGHT);
    }

    public boolean isLeftRetracted() {
        return (leftEncoder.getPosition() < ClimberConstants.MIN_HEIGHT);
    }

    public boolean isRightRetracted() {
        return (rightEncoder.getPosition() < ClimberConstants.MIN_HEIGHT);
    }

    public double getClimberHeight() {
        return (leftEncoder.getPosition() + rightEncoder.getPosition()) / 2;
    }

    public double getLeftEncoderPosition() {
        // gets position in inches
        return leftEncoder.getPosition();
    }

    public double getRightEncoderPosition() {
        // gets position in inches
        return rightEncoder.getPosition();
    }

}
