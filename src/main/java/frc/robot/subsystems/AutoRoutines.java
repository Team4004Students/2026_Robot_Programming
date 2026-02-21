// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ClimberUp;
import frc.robot.commands.IndexerRun;
import frc.robot.commands.IndexerStop;
import frc.robot.commands.IntakeDown;
import frc.robot.commands.IntakeDown;
import frc.robot.commands.IntakeUp;
import frc.robot.commands.RunIntake;
import frc.robot.commands.RunIntake;
import frc.robot.commands.ShooterRun;
import frc.robot.commands.ShooterStop;
import frc.robot.commands.StopIntake;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.IntakePosition;
import frc.robot.subsystems.Shooter;

public class AutoRoutines extends SubsystemBase {
  /** Creates a new AutoRoutines. */
    private final AutoFactory m_factory;
    private final Climber m_climber;
    private final Intake m_intake;
    private final Shooter m_shooter;
    private final Indexer m_indexer;
    private final IntakePosition m_intakepos;

    public AutoRoutines(AutoFactory factory, Climber climber, Intake intake, Shooter shooter, Indexer indexer, IntakePosition intakepos) {
        m_factory = factory;
        m_climber = climber;
        m_intake = intake;
        m_shooter = shooter;
        m_indexer = indexer;
        m_intakepos = intakepos;
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

    public AutoRoutine POS1PickShootClimb() {
        final AutoRoutine routine = m_factory.newRoutine("POS1PickShootClimb Auto");
        final AutoTrajectory POS1PickShootClimbPath1 = routine.trajectory("POS1PickShootClimb1");
        final AutoTrajectory POS1PickShootClimbPath2 = routine.trajectory("POS1PickShootClimb2");
        routine.active().onTrue(
            POS1PickShootClimbPath1.resetOdometry()
            .andThen(new IntakeDown(m_intakepos))
              .andThen(new RunIntake(m_intake))
               .andThen(new ShooterRun(m_shooter))
                .andThen(POS1PickShootClimbPath1.cmd())
                  .andThen(new IndexerRun(m_indexer))
                    .andThen(new WaitCommand(5.0))
                      .andThen(new IndexerStop(m_indexer))
                        .andThen(new ShooterStop(m_shooter))
                          .andThen(new StopIntake(m_intake))
                           .andThen(new IntakeUp(m_intakepos))
                            .andThen(POS1PickShootClimbPath2.cmd())
                              .andThen(new ClimberUp(m_climber))
                  
        );                                                                                                                
        return routine;
    }

     public AutoRoutine POS3loadShoot() {
        final AutoRoutine routine = m_factory.newRoutine("POS3loadShoot Auto");
        final AutoTrajectory POS3loadShootPath = routine.trajectory("POS3loadShoot");

        routine.active().onTrue(
            POS3loadShootPath.resetOdometry()
                .andThen(POS3loadShootPath.cmd())
        );                                                                                                                
        return routine;
    }

     public AutoRoutine POS2BackShootClimb() {
        final AutoRoutine routine = m_factory.newRoutine("POS2BackShootClimb Auto");
        final AutoTrajectory POS2BackShootClimbPath = routine.trajectory("POS2BackShootClimb");

        routine.active().onTrue(
            POS2BackShootClimbPath.resetOdometry()
                .andThen(POS2BackShootClimbPath.cmd())
        );                                                                                                                
        return routine;
    }

     public AutoRoutine POS1PickShoot() {
        final AutoRoutine routine = m_factory.newRoutine("POS1PickShoot Auto");
        final AutoTrajectory POS1PickShootPath = routine.trajectory("POS1PickShoot");

        routine.active().onTrue(
            POS1PickShootPath.resetOdometry()
                .andThen(POS1PickShootPath.cmd())
        );                                                                                                                
        return routine;
    }

     public AutoRoutine POS3loadShootClimb() {
        final AutoRoutine routine = m_factory.newRoutine("POS3loadShootClimb Auto");
        final AutoTrajectory POS3loadShootClimbPath = routine.trajectory("POS3loadShootClimb");

        routine.active().onTrue(
            POS3loadShootClimbPath.resetOdometry()
                .andThen(POS3loadShootClimbPath.cmd())
        );                                                                                                                
        return routine;
    }

     public AutoRoutine POS4Bump() {
        final AutoRoutine routine = m_factory.newRoutine("POS4Bump Auto");
        final AutoTrajectory POS4BumpPath = routine.trajectory("POS4Bump");

        routine.active().onTrue(
            POS4BumpPath.resetOdometry()
                .andThen(POS4BumpPath.cmd())
        );                                                                                                                
        return routine;
    }

     public AutoRoutine POS5Bump() {
        final AutoRoutine routine = m_factory.newRoutine("POS5Bump Auto");
        final AutoTrajectory POS5BumpPath = routine.trajectory("POS5Bump");

        routine.active().onTrue(
            POS5BumpPath.resetOdometry()
                .andThen(POS5BumpPath.cmd())
        );                                                                                                                
        return routine;
    }

     public AutoRoutine ScrewYou() {
        final AutoRoutine routine = m_factory.newRoutine("ScrewYou Auto");
        final AutoTrajectory ScrewYouPath = routine.trajectory("ScrewYou");

        routine.active().onTrue(
            ScrewYouPath.resetOdometry()
              .andThen(new IntakeDown(m_intakepos))
               .andThen(new RunIntake(m_intake))
                .andThen(ScrewYouPath.cmd())
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
    @Override
    public void periodic() {
      // This method will be called once per scheduler run
    }
}
