// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.ClimberUp;
import frc.robot.commands.IndexerRun;
import frc.robot.commands.IndexerStop;
import frc.robot.commands.IntakeDown;
import frc.robot.commands.IntakeReset;
import frc.robot.commands.IntakeBumpPosition;
import frc.robot.commands.IntakeShootPosition;
import frc.robot.commands.IntakeUp;
import frc.robot.commands.IntakeRun;
import frc.robot.commands.ShooterRun;
import frc.robot.commands.ShooterStop;
import frc.robot.generated.TunerConstants;
import frc.robot.commands.IntakeStop;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.IntakePosition;
import frc.robot.subsystems.Shooter;

public class AutoRoutines {
  /** Creates a new AutoRoutines. */
    private final AutoFactory m_factory;
    private final CommandSwerveDrivetrain m_drivetrain;
    private final Climber m_climber;
    private final Intake m_intake;
    private final Shooter m_shooter;
    private final Indexer m_indexer;
    private final IntakePosition m_intakepos;
    private Field2d field = new Field2d();

    public AutoRoutines(AutoFactory factory, CommandSwerveDrivetrain drivetrain, Climber climber, Intake intake, Shooter shooter, Indexer indexer, IntakePosition intakepos) {
        m_factory = factory;
        m_drivetrain = drivetrain;
        m_climber = climber;
        m_intake = intake;
        m_shooter = shooter;
        m_indexer = indexer;
        m_intakepos = intakepos;

        SmartDashboard.putData("Auto Field", field);
    }

    public AutoRoutine doNothingAuto() {
        final AutoRoutine routine = m_factory.newRoutine("Do Nothing Auto");

        routine.active().onTrue(
            new WaitCommand(5)
        );
        return routine;
    }

    public AutoRoutine simplePathAuto() {
        final AutoRoutine routine = m_factory.newRoutine("SimplePath Auto");
        final AutoTrajectory simplePath = routine.trajectory("SimplePath");

        routine.active().onTrue(
            simplePath.resetOdometry()
            .andThen(simplePath.cmd())
        );
        return routine;
    }

    public AutoRoutine muskegonBackShoot() {
        final double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
        final double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

        final AutoRoutine routine = m_factory.newRoutine("MUSKEGON BACK UP AND SHOOT Auto");
        final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

        routine.active().onTrue(
            m_drivetrain.runOnce(() -> m_drivetrain.seedFieldCentric(Rotation2d.kZero))
            // Then slowly drive forward (away from us) for 5 seconds.
            .andThen(m_drivetrain.applyRequest(() ->
                drive.withVelocityX(-0.5)
                    .withVelocityY(0)
                    .withRotationalRate(0)
            ).withTimeout(5.0))
            // Finally idle for the rest of auton
            .andThen(m_drivetrain.applyRequest(() -> new SwerveRequest.Idle()))
            .andThen(new IntakeDown(m_intakepos))
            .andThen(new ShooterRun(m_shooter))
            .andThen(new WaitUntilCommand(m_shooter::atSpeed))
            .andThen(new IndexerRun(m_indexer))
            .andThen(new WaitCommand(5.0))
            .andThen(new IndexerStop(m_indexer))
            .andThen(new ShooterStop(m_shooter))
            .andThen(new IntakeUp(m_intakepos))
        );
        return routine;
    }

    public AutoRoutine POS1PickShootClimb() {
        final AutoRoutine routine = m_factory.newRoutine("POS1PickShootClimb Auto");
        final AutoTrajectory POS1PickShootClimbPath1 = routine.trajectory("POS1PickShootClimb1");
        final AutoTrajectory POS1PickShootClimbPath2 = routine.trajectory("POS1PickShootClimb2");
        routine.active().onTrue(
            POS1PickShootClimbPath1.resetOdometry()
            .andThen(new IntakeDown(m_intakepos))
            .andThen(new IntakeRun(m_intake))
            .andThen(new ShooterRun(m_shooter))
            .andThen(POS1PickShootClimbPath1.cmd())
            .andThen(new IntakeStop(m_intake))
            .andThen(new IndexerRun(m_indexer))
            .andThen(new WaitCommand(5.0))
            .andThen(new IndexerStop(m_indexer))
            .andThen(new ShooterStop(m_shooter))
            .andThen(new IntakeUp(m_intakepos))
            .andThen(POS1PickShootClimbPath2.cmd().withTimeout(2.0))
            .andThen(new ClimberUp(m_climber).withTimeout(4.0))
        );                                                                                                                
        return routine;
    }

    public AutoRoutine POS3loadShoot() {
        final AutoRoutine routine = m_factory.newRoutine("POS3loadShoot Auto");
        final AutoTrajectory POS3loadShootPath1 = routine.trajectory("POS3loadShoot1");
        final AutoTrajectory POS3loadShootPath2 = routine.trajectory("POS3loadShoot2");
        routine.active().onTrue(
            POS3loadShootPath1.resetOdometry()
            .andThen(POS3loadShootPath1.cmd())
            .andThen(new IntakeDown(m_intakepos))
            .andThen(new WaitCommand(5.0))
            .andThen(new ShooterRun(m_shooter))
            .andThen(POS3loadShootPath2.cmd())
            .andThen(new IndexerRun(m_indexer))
        );                                                                                                                
        return routine;
    }

    public AutoRoutine POS2BackShootClimb() {
        final AutoRoutine routine = m_factory.newRoutine("POS2BackShootClimb Auto");
        final AutoTrajectory POS2BackShootClimbPath1 = routine.trajectory("POS2BackShootClimb1");
        final AutoTrajectory POS2BackShootClimbPath2 = routine.trajectory("POS2BackShootClimb2");
        
        field.getObject("traj").setPoses(POS2BackShootClimbPath1.getFinalPose().get());
        
        routine.active().onTrue(
            POS2BackShootClimbPath1.resetOdometry()
            .andThen(POS2BackShootClimbPath1.cmd())
            .andThen(new IntakeDown(m_intakepos))
            .andThen(new ShooterRun(m_shooter))
            .andThen(new WaitUntilCommand(m_shooter::atSpeed))
            .andThen(new IndexerRun(m_indexer))
            .andThen(new WaitCommand(5.0))
            .andThen(new IndexerStop(m_indexer))
            .andThen(new ShooterStop(m_shooter))
            .andThen(new IntakeUp(m_intakepos))
            //.andThen(POS2BackShootClimbPath2.cmd().withTimeout(1.5))
           // .andThen(new ClimberUp(m_climber).withTimeout(4.0))
        );                                                                                                                
        return routine;
    }

    public AutoRoutine POS1PickShoot() {
        final AutoRoutine routine = m_factory.newRoutine("POS1PickShoot Auto");
        final AutoTrajectory POS1PickShootPath1 = routine.trajectory("POS1PickShoot1");
        final AutoTrajectory POS1PickShootPath2 = routine.trajectory("POS1PickShoot2");

        routine.active().onTrue(
            POS1PickShootPath1.resetOdometry()
            .andThen(new IntakeDown(m_intakepos))
            .andThen(new IntakeRun(m_intake))
            .andThen(POS1PickShootPath1.cmd())
            .andThen(new IntakeStop(m_intake))
            .andThen(new ShooterRun(m_shooter))
            .andThen(POS1PickShootPath2.cmd())
            .andThen(new WaitUntilCommand(m_shooter::atSpeed))
            .andThen(new IndexerRun(m_indexer))
        );                                                                                                                
        return routine;
    }

    public AutoRoutine POS3loadShootClimb() {
        final AutoRoutine routine = m_factory.newRoutine("POS3loadShootClimb Auto");
        final AutoTrajectory POS3loadShootClimbPath1 = routine.trajectory("POS3loadShootClimb1");
        final AutoTrajectory POS3loadShootClimbPath2 = routine.trajectory("POS3loadShootClimb2");
        final AutoTrajectory POS3loadShootClimbPath3 = routine.trajectory("POS3loadShootClimb3");

        routine.active().onTrue(
        POS3loadShootClimbPath1.resetOdometry()
            .andThen(POS3loadShootClimbPath1.cmd())
            .andThen(new IntakeDown(m_intakepos))
            .andThen(new WaitCommand(5.0))
            .andThen(new ShooterRun(m_shooter))
            .andThen(POS3loadShootClimbPath2.cmd())
            .andThen(new IndexerRun(m_indexer))
            .andThen(new WaitCommand(5.0))
            .andThen(new IndexerStop(m_indexer))
            .andThen(new ShooterStop(m_shooter))
            .andThen(new IntakeUp(m_intakepos))
            .andThen(POS3loadShootClimbPath3.cmd().withTimeout(2.0))
            .andThen(new ClimberUp(m_climber).withTimeout(4.0))
        );                                                                                                                
        return routine;
    }

    public AutoRoutine POS4Bump() {
        final AutoRoutine routine = m_factory.newRoutine("POS4Bump Auto");
        final AutoTrajectory POS4BumpPath1 = routine.trajectory("POS4Bump1");
        final AutoTrajectory POS4BumpPath2 = routine.trajectory("POS4Bump2");
        final AutoTrajectory POS4BumpPath3 = routine.trajectory("POS4Bump3");

        routine.active().onTrue(
            POS4BumpPath1.resetOdometry()
            .andThen(new IntakeDown(m_intakepos))
            .andThen(new ShooterRun(m_shooter))
            .andThen(POS4BumpPath1.cmd())
            .andThen(new WaitUntilCommand(m_shooter::atSpeed))
            .andThen(new IndexerRun(m_indexer))
            .andThen(new IntakeBumpPosition(m_intakepos))
            .andThen(new IntakeDown(m_intakepos))
            .andThen(new IntakeBumpPosition(m_intakepos))
            .andThen(new IntakeDown(m_intakepos))
            .andThen(new WaitCommand(5.0))
            .andThen(new IndexerStop(m_indexer))
            .andThen(new ShooterStop(m_shooter))
            .andThen(new IntakeUp(m_intakepos))
            .andThen(POS4BumpPath2.cmd())
            .andThen(new IntakeDown(m_intakepos))
            .andThen(new IntakeRun(m_intake))
            .andThen(POS4BumpPath3.cmd())
        );                                                                                                                
        return routine;
    }

    public AutoRoutine POS5Bump() {
        final AutoRoutine routine = m_factory.newRoutine("POS5Bump Auto");
        final AutoTrajectory POS5BumpPath1 = routine.trajectory("POS5Bump1");
        final AutoTrajectory POS5BumpPath2 = routine.trajectory("POS5Bump2");
        final AutoTrajectory POS5BumpPath3 = routine.trajectory("POS5Bump3");

        routine.active().onTrue(
            POS5BumpPath1.resetOdometry()
            .andThen(new IntakeDown(m_intakepos))
            .andThen(new ShooterRun(m_shooter))
            .andThen(POS5BumpPath1.cmd())
            .andThen(new WaitUntilCommand(m_shooter::atSpeed))
            .andThen(new IndexerRun(m_indexer))
            .andThen(new WaitCommand(5.0))
            .andThen(new IndexerStop(m_indexer))
            .andThen(new ShooterStop(m_shooter))
            .andThen(new IntakeUp(m_intakepos))
            .andThen(POS5BumpPath2.cmd())
            .andThen(new IntakeDown(m_intakepos))
            .andThen(new IntakeRun(m_intake))
            .andThen(POS5BumpPath3.cmd())
        );                                                                                                                
        return routine;
    }

    public AutoRoutine ScrewYou() {
        final AutoRoutine routine = m_factory.newRoutine("ScrewYou Auto");
        final AutoTrajectory ScrewYouPath1 = routine.trajectory("ScrewYou1");
        final AutoTrajectory ScrewYouPath2 = routine.trajectory("ScrewYou2");

        routine.active().onTrue(
            ScrewYouPath1.resetOdometry()
            .andThen(ScrewYouPath1.cmd())
            .andThen(new IntakeDown(m_intakepos))
            .andThen(new IntakeRun(m_intake))
            .andThen(ScrewYouPath2.cmd())
        );                                                                                                                
        return routine;
    }

    public AutoRoutine Paul() {
        final AutoRoutine routine = m_factory.newRoutine("Paul Auto");
        final AutoTrajectory PaulPath = routine.trajectory("Paul");

        routine.active().onTrue(
            PaulPath.resetOdometry()
            .andThen(PaulPath.cmd())
        );                                                                                                                
        return routine;
    }

    public AutoRoutine simpleMultiPathAuto() {
        final AutoRoutine routine = m_factory.newRoutine("SimpleMultiPath Auto");
        final AutoTrajectory scoreCoralMiddle = routine.trajectory("ScoreCoralMiddle");
        final AutoTrajectory goToFeederStationFromSideTwo = routine.trajectory("GetToFeederStationFromSideTwo");
        final AutoTrajectory scoreCoralSideThreeFromFeederStation = routine.trajectory("ScoreCoralSideThreeFromFeederStation");
        final AutoTrajectory goToFeederStationFromSideThree = routine.trajectory("GetToFeederStationFromSideThree");

        routine.active().onTrue(
            scoreCoralMiddle.resetOdometry()
            .andThen(scoreCoralMiddle.cmd())
            .andThen(goToFeederStationFromSideTwo.cmd())
            .andThen(scoreCoralSideThreeFromFeederStation.cmd())
            .andThen(goToFeederStationFromSideThree.cmd())
            .andThen(scoreCoralSideThreeFromFeederStation.cmd())
        );
        return routine;
    }
    
    public AutoRoutine POS6TrenchShootAuto() {
        final AutoRoutine routine = m_factory.newRoutine("POS6TrenchShoot Auto");
        final AutoTrajectory POS6TrenchShootPath = routine.trajectory("POS6TrenchShoot");

        // region Current Issue
        routine.active().onTrue(
            POS6TrenchShootPath.resetOdometry()
            .andThen(new IntakeDown(m_intakepos))
            .andThen(new IntakeRun(m_intake))
            .andThen(POS6TrenchShootPath.cmd())
            .andThen(new ShooterRun(m_shooter))
            .andThen(new WaitUntilCommand(m_shooter::atSpeed))
            .andThen(new IndexerRun(m_indexer))
            .andThen(new IntakeShootPosition(m_intakepos))
            .andThen(new IntakeDown(m_intakepos))
            .andThen(new IntakeShootPosition(m_intakepos))
            .andThen(new IntakeDown(m_intakepos))
            .andThen(new WaitCommand(5))
            .andThen(new IndexerStop(m_indexer))
            .andThen(new ShooterStop(m_shooter))
           
        );      
        // endregion Current Issue                                                                                                          
        return routine;
    }
    
    public AutoRoutine POS7TrenchShootAuto() {
        final AutoRoutine routine = m_factory.newRoutine("POS7TrenchShoot Auto");
        final AutoTrajectory POS7TrenchShootPath = routine.trajectory("POS7TrenchShoot");

        // region Currently Working
        routine.active().onTrue(
            POS7TrenchShootPath.resetOdometry()
            .andThen(new IntakeReset(m_intakepos))
            .andThen(new IntakeDown(m_intakepos))
            .andThen(new IntakeRun(m_intake))
            .andThen(POS7TrenchShootPath.cmd())
            .andThen(new ShooterRun(m_shooter))
            .andThen(new WaitUntilCommand(m_shooter::atSpeed))
            .andThen(new IndexerRun(m_indexer))
            .andThen(new IntakeShootPosition(m_intakepos))
            .andThen(new IntakeDown(m_intakepos))
            .andThen(new IntakeShootPosition(m_intakepos))
            .andThen(new IntakeDown(m_intakepos))
            .andThen(new WaitCommand(5))
            .andThen(new IndexerStop(m_indexer))
            .andThen(new ShooterStop(m_shooter))
            
        );   
        // endregion Currently Working                                                                                                             
        return routine;
    }
        public AutoRoutine POS7TrenchShootUnderAuto() {
        final AutoRoutine routine = m_factory.newRoutine("POS7TrenchShootUnder Auto");
        final AutoTrajectory POS7TrenchShootUnderPath = routine.trajectory("POS7TrenchShootUnder");

        routine.active().onTrue(
            POS7TrenchShootUnderPath.resetOdometry()
            .andThen(new IntakeReset(m_intakepos))
            .andThen(new IntakeDown(m_intakepos))
            .andThen(new IntakeRun(m_intake))
            .andThen(new ShooterRun(m_shooter))
            .andThen(POS7TrenchShootUnderPath.cmd())
            .andThen(new WaitUntilCommand(m_shooter::atSpeed))
            .andThen(new IndexerRun(m_indexer))
            .andThen(new IntakeShootPosition(m_intakepos))
            .andThen(new IntakeDown(m_intakepos))
            .andThen(new IntakeShootPosition(m_intakepos))
            .andThen(new IntakeDown(m_intakepos))
            .andThen(new WaitCommand(5))
            .andThen(new IndexerStop(m_indexer))
            .andThen(new ShooterStop(m_shooter))
            
        );                                                                                                                
        return routine;
    }
      public AutoRoutine POS6TrenchShootx2Auto() {
        final AutoRoutine routine = m_factory.newRoutine("POS6TrenchShootx2 Auto");
        final AutoTrajectory POS6TrenchShootx2Path1 = routine.trajectory("POS6TrenchShootx2P1");
        final AutoTrajectory POS6TrenchShootx2Path2 = routine.trajectory("POS6TrenchShootx2P2");

        routine.active().onTrue(
            POS6TrenchShootx2Path1.resetOdometry()
            .andThen(new IntakeReset(m_intakepos))
            .andThen(new IntakeDown(m_intakepos))
            .andThen(new IntakeRun(m_intake))
            .andThen(new ShooterRun(m_shooter))
            .andThen(POS6TrenchShootx2Path1.cmd())
            .andThen(new WaitUntilCommand(m_shooter::atSpeed))
            .andThen(new IndexerRun(m_indexer))
            .andThen(new IntakeShootPosition(m_intakepos)
            .andThen(new IntakeDown(m_intakepos))
            .andThen(new IndexerStop(m_indexer))
            //.andThen(new ShooterStop(m_shooter))
            .andThen(new WaitCommand(1))
            .andThen(new IntakeRun(m_intake))
            .andThen(POS6TrenchShootx2Path2.cmd())
            .andThen(new ShooterRun(m_shooter))
            .andThen(new WaitUntilCommand(m_shooter::atSpeed))
            .andThen(new IndexerRun(m_indexer))
            .andThen(new IntakeShootPosition(m_intakepos))
            //.andThen(new WaitCommand(2))
            //.andThen(new IndexerStop(m_indexer))
            //.andThen(new ShooterStop(m_shooter))
            )
            );                                                                                                                
        return routine;
    }
      public AutoRoutine POS7TrenchShootx2Auto() {
        final AutoRoutine routine = m_factory.newRoutine("POS7TrenchShootx2 Auto");
        final AutoTrajectory POS7TrenchShootx2Path1 = routine.trajectory("POS7TrenchShootx2P1");
        final AutoTrajectory POS7TrenchShootx2Path2 = routine.trajectory("POS7TrenchShootx2P2");

        routine.active().onTrue(
            POS7TrenchShootx2Path1.resetOdometry()
            .andThen(new IntakeReset(m_intakepos))
            .andThen(new IntakeDown(m_intakepos).withTimeout(1))
            .andThen(new IntakeRun(m_intake))
            .andThen(new ShooterRun(m_shooter))
            .andThen(POS7TrenchShootx2Path1.cmd())
            .andThen(new WaitUntilCommand(m_shooter::atSpeed))
            .andThen(new IndexerRun(m_indexer))
            .andThen(new IntakeShootPosition(m_intakepos).withTimeout(1))
            .andThen(new IntakeDown(m_intakepos).withTimeout(1))
            .andThen(new IntakeShootPosition(m_intakepos).withTimeout(1))
            .andThen(new IntakeDown(m_intakepos).withTimeout(1))
            .andThen(new IntakeShootPosition(m_intakepos).withTimeout(1))
            .andThen(new IntakeDown(m_intakepos).withTimeout(1))
            .andThen(new IntakeShootPosition(m_intakepos).withTimeout(1))
            .andThen(new IntakeDown(m_intakepos).withTimeout(1))
            .andThen(new IntakeShootPosition(m_intakepos).withTimeout(1))
            .andThen(new IntakeDown(m_intakepos).withTimeout(1))
            .andThen(new IntakeShootPosition(m_intakepos).withTimeout(1))
            .andThen(new IntakeDown(m_intakepos).withTimeout(1))
            .andThen(new IntakeShootPosition(m_intakepos).withTimeout(1))
            .andThen(new IntakeDown(m_intakepos).withTimeout(1))
            .andThen(new IntakeShootPosition(m_intakepos).withTimeout(1)) 
            .andThen(new IntakeDown(m_intakepos).withTimeout(1))
            .andThen(new IndexerStop(m_indexer))
           // .andThen(new ShooterStop(m_shooter))
            .andThen(new WaitCommand(.5))
            .andThen(new IntakeRun(m_intake))
            .andThen(POS7TrenchShootx2Path2.cmd())
            .andThen(new ShooterRun(m_shooter))
            .andThen(new WaitUntilCommand(m_shooter::atSpeed))
            .andThen(new IndexerRun(m_indexer).withTimeout(1))
            .andThen(new IntakeShootPosition(m_intakepos).withTimeout(1))
            .andThen(new IntakeDown(m_intakepos).withTimeout(1))
            .andThen(new IntakeShootPosition(m_intakepos).withTimeout(1))
            .andThen(new IntakeDown(m_intakepos).withTimeout(1))
            .andThen(new IntakeShootPosition(m_intakepos).withTimeout(1))
            .andThen(new IntakeDown(m_intakepos).withTimeout(1))
            .andThen(new IntakeShootPosition(m_intakepos).withTimeout(1))
            .andThen(new IntakeDown(m_intakepos).withTimeout(1))
            .andThen(new IntakeShootPosition(m_intakepos).withTimeout(1))
            .andThen(new IntakeDown(m_intakepos).withTimeout(1))
            .andThen(new IntakeShootPosition(m_intakepos).withTimeout(1)) 
            .andThen(new IntakeDown(m_intakepos).withTimeout(1))
            .andThen(new IntakeShootPosition(m_intakepos).withTimeout(1))
            .andThen(new IntakeDown(m_intakepos).withTimeout(1))
            .andThen(new IntakeShootPosition(m_intakepos).withTimeout(1))
            .andThen(new IntakeDown(m_intakepos).withTimeout(1))
           
        );                                                                                                                
        return routine;
    }
    public AutoRoutine ScrewYouTrench() {
        final AutoRoutine routine = m_factory.newRoutine("ScrewYouTrench Auto");
        final AutoTrajectory ScrewYouTrenchPath1 = routine.trajectory("ScrewYouTrench1");
        final AutoTrajectory ScrewYouTrenchPath2 = routine.trajectory("ScrewYouTrench2");

        routine.active().onTrue(
            ScrewYouTrenchPath1.resetOdometry()
            .andThen(new IntakeDown(m_intakepos))
            .andThen(ScrewYouTrenchPath1.cmd())
            .andThen(new IntakeRun(m_intake))
            .andThen(ScrewYouTrenchPath2.cmd())
        );                                                                                                                
        return routine;
    }
    public AutoRoutine TestAutoSquare() {
        final AutoRoutine routine = m_factory.newRoutine("Test Auto");
        final AutoTrajectory TestPath = routine.trajectory("Test");
        
        routine.active().onTrue(TestPath.resetOdometry()
        .andThen(TestPath.cmd())
        );
        return routine;
    }
}
