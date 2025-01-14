package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.utils.SimEncoder;

public class SimShooter implements ShooterIO {

    public static SimEncoder shooterEncoderSim;
    private DCMotorSim shooterMotorSim;
    private PIDController pidController;

    public SimShooter() {
        shooterEncoderSim = new SimEncoder("shooter");
        shooterMotorSim = new DCMotorSim(DCMotor.getNeo550(1), 1, 1);
        SmartDashboard.putNumber("shooter current sim", 0);
        SmartDashboard.putNumber("shooter sim velocity", 0);
        pidController = new PIDController(1, 0,0);
    }

    public void setMotor(double speed) {
        shooterMotorSim.setInput(speed);
    }

    public double getCurrent() {
        return SmartDashboard.getNumber("shooter current sim", -100);
    }

    public double getEncoderSpeed() {
        return SmartDashboard.getNumber("shooter sim velocity", -100);
    }

    public void setCurrentLimit(int current) {
        SmartDashboard.getNumber("shooter current sim", current);
    }

    public void periodicUpdate() {
        SmartDashboard.putNumber("shooter/current (A)", getCurrent());
    }

    public void setSpeed(double speed) {
        pidController.calculate(getEncoderSpeed(), speed);
    }
}
