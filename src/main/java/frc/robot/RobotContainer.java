// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.List;
import frc.robot.subsystems.*;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberReal;
import frc.robot.subsystems.shooter.RealShooter;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
        // The robot's subsystems
        // private final DriveSubsystem m_robotDrive = new DriveSubsystem();
        // private static Gyro m_gyro = new Gyro();

        public static Shooter shooter;
        public static Climber climber;

        // defaults to field oriented driving and defaults operator control to teleop mode
        public boolean fieldOrientedDrive = true;
        public static boolean isInClimberMode = false; 

        // The driver's controller and the operator's controller
        XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
        CommandXboxController m_operatorController = new CommandXboxController(OIConstants.kOperatorControllerPort);

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                // Configure the button bindings
                SmartDashboard.putNumber("Shoot speed", SmartDashboard.getNumber("Shoot speed", 0));
                // setUpShooter();
                setUpClimber();
                configureButtonBindings();

                shooter.setDefaultCommand(
                                new InstantCommand(
                                                () -> shooter.setMotor(SmartDashboard.getNumber("Shoot speed", 0)),
                                                shooter));

                // Configure default commands
                // m_robotDrive.setDefaultCommand(
                // // The left stick controls translation of the robot.
                // // Turning is controlled by the X axis of the right stick.
                // new RunCommand(
                // () -> m_robotDrive.drive(
                // -MathUtil.applyDeadband(m_driverController.getLeftY(),
                // OIConstants.kDriveDeadband),
                // -MathUtil.applyDeadband(m_driverController.getLeftX(),
                // OIConstants.kDriveDeadband),
                // -MathUtil.applyDeadband(m_driverController.getRightX(),
                // OIConstants.kDriveDeadband),
                // fieldOrientedDrive, false),
                // m_robotDrive));
                // new RunCommand(
                // () -> m_robotDrive.drive(
                // -MathUtil.applyDeadband(m_driverController.getLeftY(),
                // OIConstants.kDriveDeadband),
                // 0,
                // 0,
                // fieldOrientedDrive, false),
                // m_robotDrive));
        }

        /**
         * Use this method to define your button->command mappings. Buttons can be
         * created by
         * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
         * subclasses ({@link
         * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
         * passing it to a
         * {@link JoystickButton}.
         */
        private void configureButtonBindings() {
                // right bumper?
                // new JoystickButton(m_driverController, XboxController.Button.kX.value)
                // .whileTrue(new RunCommand(
                // () -> m_robotDrive.setX(),
                // m_robotDrive));

                m_operatorController.leftTrigger().and(()->isInClimberMode).whileTrue(new RunCommand(
                                () -> climber.setLeftSpeed(0.2),climber)

                );
                m_operatorController.rightTrigger().and(()->isInClimberMode).whileTrue(new RunCommand(
                                () -> climber.setRightSpeed(0.2),climber)

                );
                m_operatorController.leftBumper().and(()->isInClimberMode).whileTrue(new RunCommand(
                                () -> climber.setLeftSpeed(-0.2),climber)

                );
                m_operatorController.rightBumper().and(()->isInClimberMode).whileTrue(new RunCommand(
                                () -> climber.setRightSpeed(-0.2),climber)

                );
                m_operatorController.x().onTrue(new InstantCommand(()->isInClimberMode = !isInClimberMode,climber)
                );

                m_operatorController.b().and(()->isInClimberMode).onTrue(new ParallelCommandGroup(
                        new InstantCommand(()-> climber.setLeftMotor(ClimberConstants.MAX_HEIGHT - 0.1),climber),
                        new InstantCommand(()-> climber.setRightMotor(ClimberConstants.MAX_HEIGHT - 0.1))
                        )

                );
                m_operatorController.a().and(()->isInClimberMode).onTrue(new ParallelCommandGroup(
                        new InstantCommand(()-> climber.setLeftMotor(ClimberConstants.MIN_HEIGHT + 0.1),climber),
                        new InstantCommand(()-> climber.setRightMotor(ClimberConstants.MIN_HEIGHT + 0.1))
                        )

                );

                // new JoystickButton(m_driverController, XboxController.Button.kY.value)
                // .whileTrue(new RunCommand(
                // () -> m_robotDrive.setZero(),
                // m_robotDrive));

                new JoystickButton(m_driverController, XboxController.Button.kA.value).onTrue(
                                new InstantCommand(
                                                () -> fieldOrientedDrive = !fieldOrientedDrive));

                // new JoystickButton(m_driverController, XboxController.Button.kB.value)
                // .onTrue(new InstantCommand(
                // () -> m_gyro.resetYaw(), m_gyro));

                new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value)
                                .whileTrue(new InstantCommand(
                                                () -> shooter.setMotor(0)));
        }

        // private void setUpShooter() {

        //         ShooterIO shooterIO;

        //         shooterIO = new RealShooter();

        //         shooter = new Shooter(shooterIO);
        // }

        private void setUpClimber() {
                ClimberIO climberIO = new ClimberReal(); 
                climber = new Climber(climberIO);
        }
        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        // public Command getAutonomousCommand() {
        // // Create config for trajectory
        // TrajectoryConfig config = new TrajectoryConfig(
        // AutoConstants.kMaxSpeedMetersPerSecond,
        // AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // // Add kinematics to ensure max speed is actually obeyed
        // .setKinematics(DriveConstants.kDriveKinematics);

        // // An example trajectory to follow. All units in meters.
        // Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // // Start at the origin facing the +X direction
        // new Pose2d(0, 0, new Rotation2d(0)),
        // // Pass through these two interior waypoints, making an 's' curve path
        // List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // // End 3 meters straight ahead of where we started, facing forward
        // new Pose2d(3, 0, new Rotation2d(0)),
        // config);

        // var thetaController = new ProfiledPIDController(
        // AutoConstants.kPThetaController, 0, 0,
        // AutoConstants.kThetaControllerConstraints);
        // thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // SwerveControllerCommand swerveControllerCommand = new
        // SwerveControllerCommand(
        // exampleTrajectory,
        // m_robotDrive::getPose, // Functional interface to feed supplier
        // DriveConstants.kDriveKinematics,

        // // Position controllers
        // new PIDController(AutoConstants.kPXController, 0, 0),
        // new PIDController(AutoConstants.kPYController, 0, 0),
        // thetaController,
        // m_robotDrive::setModuleStates,
        // m_robotDrive);

        // // Reset odometry to the starting pose of the trajectory.
        // m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

        // // Run path following command, then stop at the end.
        // return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0,
        // false, false));
        // }
}
