package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    private ClimberIO climberIO;

    public Climber(ClimberIO io) {

        climberIO = io;
    }

    public void setLeftSpeed(double speed) {
        climberIO.setLeftSpeed(speed);
    }

    public void setRightSpeed(double speed) {
        climberIO.setRightSpeed(speed);
    }

    public void setRightMotor(double setpoint) {
        climberIO.setRightMotor(setpoint);
    }

    public void setLeftMotor(double setpoint) {
        climberIO.setLeftMotor(setpoint);
    }

    public void peridoic(){
        climberIO.periodicUpdate(); 
    }

}