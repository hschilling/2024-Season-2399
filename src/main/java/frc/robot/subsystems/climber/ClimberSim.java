package frc.robot.subsystems.climber;

public class ClimberSim implements ClimberIO {
    public void setLeftSpeed(double speed) {

    }

    public void setLeftMotor(double setpoint) {

    }

    public void setRightSpeed(double speed) {

    }

    public void setRightMotor(double setpoint) {

    }

    public boolean isInClimberMode(){
        return true;
    }

    public boolean isLeftExtended() {
        return true;
    }

    public boolean isRightExtended() {
        return true;
    }

    public boolean isLeftRetracted() {
        return true;
    }

    public boolean isRightRetracted() {
        return true;
    }

    public double getLeftEncoderPosition() {
        return 0.0;
    }

    public double getRightEncoderPosition() {
        return 0.0;
    }

    public boolean isLeftSideStalling() {
        return true;
    }

    public boolean isRightSideStalling() {
        return true;
    }

    public void periodicUpdate() {

    }
}
