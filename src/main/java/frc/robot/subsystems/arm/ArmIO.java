package frc.robot.subsystems.arm;

public interface ArmIO {
    public double getEncoderPosition();

    public double getEncoderSpeed();

    public void setSpeed(double speed);

    public void periodicUpdate();

    public double getArmCurrent();

    public double getAbsoluteEncoderPosition();

    public void setEncoderPosition(double angle);

    public double getSpeedFromArmHeight();
}
