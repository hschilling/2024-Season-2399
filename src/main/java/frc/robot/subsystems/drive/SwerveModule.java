package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.drive.SwerveModuleIO.SwerveModuleIOInputs;

public class SwerveModule {

    private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

    double chassisAngularOffset;

    SwerveModuleIO io;

    private final SwerveModuleIOInputs inputs = new SwerveModuleIOInputs();

    private String name;

    public SwerveModule(SwerveModuleIO io) {

        this.io = io;
        this.name = io.getName();
        io.setDriveEncoderPosition(0);
        m_desiredState.angle = new Rotation2d(getTurnEncoderPosition());

    }

    public void setDriveEncoderPosition(double position) {
        io.setDriveEncoderPosition(position);
    }

    public double getDriveEncoderPosition() {
        return io.getDriveEncoderPosition();
    }

    public double getDriveEncoderSpeedMPS() {
        return io.getDriveEncoderSpeedMPS();
    }

    public double getTurnEncoderPosition() {
        return io.getTurnEncoderPosition();
    }

    public void resetEncoders() {
        io.setDriveEncoderPosition(0);
    }

    public double getDriveBusVoltage() {
        return io.getDriveBusVoltage();
    }

    public double getDriveOutput() {
        return io.getDriveOutput();
    }

    public double getTurnBusVoltage() {
        return io.getTurnBusVoltage();
    }

    public void updateInputs() {
        io.updateInputs(inputs);
    }

    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    public SwerveModuleState getState() {
        // Apply chassis angular offset to the encoder position to get the position
        // relative to the chassis.
        if (name == "front left") {
            // System.out.println(name + " getDriveEncoderSpeedMPS() = " + getDriveEncoderSpeedMPS());
        }
        return new SwerveModuleState(getDriveEncoderSpeedMPS(),
                new Rotation2d((getTurnEncoderPosition()) - io.getChassisAngularOffset()));
    }

    /**
     * Returns the current position of the module.
     *
     * @return The current position of the module.
     */
    public SwerveModulePosition getPosition() {
        // Apply chassis angular offset to the encoder position to get the position
        // relative to the chassis.
        SmartDashboard.putNumber(io.getName() + " getDriveEncoderPosition in SwerveModule", getDriveEncoderPosition());

        SmartDashboard.putNumber(io.getName() + " getTurnEncoderPosition in SwerveModule", getTurnEncoderPosition());
        return new SwerveModulePosition(
                getDriveEncoderPosition(),
                new Rotation2d(getTurnEncoderPosition() - io.getChassisAngularOffset()));
    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        // Apply chassis angular offset to the desired state.
        SwerveModuleState correctedDesiredState = new SwerveModuleState();
        correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
        SmartDashboard.putNumber("drive/" + name + "desired angle before optimize", desiredState.angle.getRadians());
        correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(io.getChassisAngularOffset()));

        // Optimize the reference state to avoid spinning further than 90 degrees.

        SwerveModuleState optimizedDesiredState =
        SwerveModuleState.optimize(correctedDesiredState,
        new Rotation2d(getTurnEncoderPosition()));


        io.setDesiredDriveSpeedMPS(optimizedDesiredState.speedMetersPerSecond);
        io.setDesiredTurnAngle(optimizedDesiredState.angle.getRadians());
        SmartDashboard.putNumber("drive/" + name + " desired angle", optimizedDesiredState.angle.getRadians());
        SmartDashboard.putNumber( "drive/" + name + " real angle", getTurnEncoderPosition());

        m_desiredState = desiredState;
    }

}