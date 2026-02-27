// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.commands.IntakeDown;
import frc.robot.commands.ClimberDown;
import frc.robot.commands.ClimberUp;
import frc.robot.commands.IntakeUp;
import frc.robot.commands.ReverseIntake;
import frc.robot.commands.RunIntake;
import frc.robot.commands.ShooterRun;
import frc.robot.commands.ShooterStop;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakePosition;
import frc.robot.subsystems.Shooter;
import frc.robot.commands.IndexerRun;
import frc.robot.commands.IndexerStop;
import java.util.function.BooleanSupplier;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    private final double SpeedLimit = 0.25 * MaxSpeed;
    private final double TurnSpeedLimit = 0.25 * MaxSpeed;
    private final double Deadband = 0.1;
    private final double Steerdeadband = 0.05;
    private final double Exponent = 1.0;

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandJoystick driveJoystick = new CommandJoystick(0);
    private final CommandJoystick steerJoystick = new CommandJoystick(1);
    private final CommandJoystick daJoystick = new CommandJoystick(2);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final Intake intake = new Intake();
    public final IntakePosition intakePosition = new IntakePosition();
    public final Shooter shooter = new Shooter(() -> steerJoystick.button(7).getAsBoolean());
    public final Indexer indexer = new Indexer(() -> steerJoystick.button(7).getAsBoolean());
    public final Climber climber = new Climber();

    /* Path follower */
    private final AutoFactory autoFactory;
    private final AutoRoutines autoRoutines;
    private final AutoChooser autoChooser = new AutoChooser();

    public RobotContainer() {
        //INITIALIZE PATH FOLLOWING
        autoFactory = drivetrain.createAutoFactory();
        autoRoutines = new AutoRoutines(autoFactory, climber, intake, shooter, indexer, intakePosition);

        //TEST AUTOS
        //autoChooser.addRoutine("SimplePath", autoRoutines::simplePathAuto);
        //autoChooser.addRoutine("SimpleMultiPath", autoRoutines::simpleMultiPathAuto);

        //POS1 AUTOS
        autoChooser.addRoutine("POS1PickShoot Auto", autoRoutines::POS1PickShoot);
        autoChooser.addRoutine("POS1PickShootClimb Auto", autoRoutines::POS1PickShootClimb);

        //POS2 AUTOS
        autoChooser.addRoutine("POS2BackShootClimb Auto", autoRoutines::POS2BackShootClimb);

        //POS3 AUTOS
        autoChooser.addRoutine("POS3LoadShoot Auto", autoRoutines::POS3loadShoot);
        autoChooser.addRoutine("POS3LoadShootClimb Auto", autoRoutines::POS3loadShootClimb);

        //POS4 AUTOS
        autoChooser.addRoutine("POS4Bump Auto", autoRoutines::POS4Bump);
        
        //POS5 AUTOS
        autoChooser.addRoutine("POS5Bump Auto", autoRoutines::POS5Bump);
        autoChooser.addRoutine("Screw You Auto", autoRoutines::ScrewYou);
        autoChooser.addRoutine("Paul Auto", autoRoutines::Paul);


        //AUTO CHOSER
        SmartDashboard.putData("Auto Chooser", autoChooser);


        configureBindings();
    }

    private void configureBindings() {
        shooter.setDefaultCommand(new ShooterStop(shooter));
        indexer.setDefaultCommand(new IndexerStop(indexer));

        steerJoystick.button(1).whileTrue(new ClimberUp (climber));
        steerJoystick.button(2).whileTrue(new ClimberDown (climber));
        daJoystick.button(6).whileTrue(new RunIntake (intake));
        steerJoystick.button(6).whileTrue(new ReverseIntake (intake));
        daJoystick.button(3).whileTrue(new IntakeDown (intakePosition));
        daJoystick.button(4).whileTrue(new IntakeUp (intakePosition));

        steerJoystick.button(5).whileTrue(new ShooterRun(shooter));
        steerJoystick.button(5).and(shooter::atSpeed).whileTrue(new IndexerRun(indexer));

        steerJoystick.button(9).whileTrue(new ShooterRun(shooter));
        steerJoystick.button(9).and(shooter::atSpeed).and(drivetrain::isPointedAtHub).whileTrue(new IndexerRun(indexer));

        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.

        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(MathUtil.clamp(Math.pow(MathUtil.applyDeadband(-driveJoystick.getY(),Deadband),Exponent) * MaxSpeed, -SpeedLimit, SpeedLimit)) // Drive forward with negative Y (forward)
                    .withVelocityY(MathUtil.clamp(Math.pow(MathUtil.applyDeadband(-driveJoystick.getX(),Deadband),Exponent) * MaxSpeed, -SpeedLimit, SpeedLimit)) // Drive left with negative X (left)
                    .withRotationalRate(MathUtil.clamp(Math.pow(MathUtil.applyDeadband(-steerJoystick.getX(),Steerdeadband),Exponent) * MaxAngularRate, -TurnSpeedLimit, TurnSpeedLimit)) // Drive counterclockwise with negative X (left)
            )
        );

        driveJoystick.button(10).whileTrue(new RepeatCommand(drivetrain.applyRequest(() ->
                drive.withVelocityX(MathUtil.clamp(Math.pow(MathUtil.applyDeadband(-driveJoystick.getY(),Deadband),Exponent) * MaxSpeed, -SpeedLimit, SpeedLimit)) // Drive forward with negative Y (forward)
                    .withVelocityY(MathUtil.clamp(Math.pow(MathUtil.applyDeadband(-driveJoystick.getX(),Deadband),Exponent) * MaxSpeed, -SpeedLimit, SpeedLimit)) // Drive left with negative X (left)
                    .withRotationalRate(drivetrain.pointAtHub() * MaxAngularRate)) // Drive counterclockwise with negative X (left)
            )
        );

        driveJoystick.button(3).whileTrue(new RepeatCommand(drivetrain.applyRequest(() ->
                drive.withVelocityX(MathUtil.clamp(Math.pow(MathUtil.applyDeadband(-driveJoystick.getY(),Deadband),Exponent) * MaxSpeed, -SpeedLimit, SpeedLimit)) // Drive forward with negative Y (forward)
                    .withVelocityY(MathUtil.clamp(Math.pow(MathUtil.applyDeadband(-driveJoystick.getX(),Deadband),Exponent) * MaxSpeed, -SpeedLimit, SpeedLimit)) // Drive left with negative X (left)
                    .withRotationalRate(drivetrain.bumpAssist() * MaxAngularRate)) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        /*  joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        )); */



        // Reset the field-centric heading on left bumper press.
        driveJoystick.button(9).onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return autoChooser.selectedCommand();
        
        //return Commands.print("No autonomous command configured");
        
        /*
        // Simple drive forward auton
        final var idle = new SwerveRequest.Idle();
        return Commands.sequence(
            // Reset our field centric heading to match the robot
            // facing away from our alliance station wall (0 deg).
            drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
            // Then slowly drive forward (away from us) for 5 seconds.
            drivetrain.applyRequest(() ->
                drive.withVelocityX(0.5)
                    .withVelocityY(0)
                    .withRotationalRate(0)
            )
            .withTimeout(5.0),
            // Finally idle for the rest of auton
            drivetrain.applyRequest(() -> idle)
        );
        */
    }
}
