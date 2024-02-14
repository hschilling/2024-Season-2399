package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.util.LEDController;



public class LED extends SubsystemBase {
    LEDController red = new LEDController(Constants.LEDConstants.RED_CHANNEL);
    LEDController green= new LEDController(Constants.LEDConstants.GREEN_CHANNEL);
    LEDController blue = new LEDController(Constants.LEDConstants.BLUE_CHANNEL);
    LEDController white = new LEDController(Constants.LEDConstants.WHITE_CHANNEL);

    public void setColor(int r, int g, int b, int w) {
        red.set(r);
        green.set(g);
        blue.set(b);
        white.set(w); 
    }

    @Override
    public void periodic() {
        if (Intake.isIntooked) {
            this.setColor(0, 255, 0, 0);
        } 
        else if (Robot.autonomousInit) {
            this.setColor(255, 50, 200, 0);
        }

    }

}