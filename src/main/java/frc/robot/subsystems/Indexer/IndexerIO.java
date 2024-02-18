package frc.robot.subsystems.Indexer;

public interface IndexerIO {
    public void setMotor(double speed);
    public void setSpeed(double speed);
    public double getCurrent();
    public double getEncoderSpeed();
    public double getEncoderPosition();
    public void setCurrentLimit(int current);
    public void periodicUpdate();
    public void setIsIntooked(boolean intooked);
    public boolean getIsBeamBroken();
    public void setIsSensorOverriden(boolean override);
    public boolean getIsIntooked();
    public boolean getIsSensorOverriden();
}
