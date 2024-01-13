# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.

from typing import TYPE_CHECKING, List, Tuple
import math

import wpimath.controller as controller
import wpimath.trajectory as trajectory
import wpimath.geometry as geometry
import wpimath.kinematics as kinematics
from wpilib import Timer

from util import *  # type: ignore

if TYPE_CHECKING:
    from .util import *

import pytest

import commands2

TWO = int(2)
THREE = int(3)
FOUR = int(4)
SIX = int(6)
MISMATCHED_KINEMATICS_AND_ODOMETRY = int(101)
INCOMPLETE_PID_CLASSES = int(102)


class SwerveControllerCommandTestDataFixtures:
    def __init__(self, selector: int):
        self._timer = Timer()
        self._angle: geometry.Rotation2d = geometry.Rotation2d(0)

        self._kxTolerance = 1 / 12.0
        self._kyTolerance = 1 / 12.0
        self._kAngularTolerance = 1 / 12.0
        self._kWheelBase = 0.5
        self._kTrackWidth = 0.5

        # The module positions and states start empty and will be populated below in the selector
        # self._modulePositions: Tuple[kinematics.SwerveModulePosition] = []
        self._modulePositions: Tuple[kinematics.SwerveModulePosition] = []
        self._moduleStates: Tuple[kinematics.SwerveModuleState] = []

        # Setup PID controllers, but if an error test case is requested, make sure it provides
        # data that should break the command instantiation
        if selector != INCOMPLETE_PID_CLASSES:
            self._xController = controller.PIDController(0.6, 0, 0)
            self._yController = controller.PIDController(0.6, 0, 0)
            constraints = trajectory.TrapezoidProfileRadians.Constraints(
                3 * math.pi, math.pi
            )
            self._rotationController = controller.ProfiledPIDControllerRadians(
                1, 0, 0, constraints
            )

            self._holonomic = controller.HolonomicDriveController(
                self._xController, self._yController, self._rotationController
            )
        else:
            self._xController = None
            self._yController = controller.PIDController(0.6, 0, 0)
            constraints = trajectory.TrapezoidProfileRadians.Constraints(
                3 * math.pi, math.pi
            )
            self._rotationController = controller.ProfiledPIDControllerRadians(
                1, 0, 0, constraints
            )
            self._holonomic = None

        if (selector == TWO) or (selector == INCOMPLETE_PID_CLASSES):
            self._kinematics = kinematics.SwerveDrive2Kinematics(
                geometry.Translation2d(self._kWheelBase / 2, self._kTrackWidth / 2),
                geometry.Translation2d(self._kWheelBase / 2, -self._kTrackWidth / 2),
            )

            self._moduleStates.append(kinematics.SwerveModuleState(0, self._angle))
            self._moduleStates.append(kinematics.SwerveModuleState(0, self._angle))

            self._modulePositions.append(
                kinematics.SwerveModulePosition(0, self._angle)
            )
            self._modulePositions.append(
                kinematics.SwerveModulePosition(0, self._angle)
            )

            self._odometry = kinematics.SwerveDrive2Odometry(
                self._kinematics,
                self._angle,
                self._modulePositions,
                geometry.Pose2d(0, 0, geometry.Rotation2d(0)),
            )
        elif selector == THREE:
            self._kinematics = kinematics.SwerveDrive3Kinematics(
                geometry.Translation2d(self._kWheelBase / 2, self._kTrackWidth / 2),
                geometry.Translation2d(self._kWheelBase / 2, -self._kTrackWidth / 2),
                geometry.Translation2d(self._kWheelBase / 2, self._kTrackWidth / 2),
            )

            self._moduleStates.append(kinematics.SwerveModuleState(0, self._angle))
            self._moduleStates.append(kinematics.SwerveModuleState(0, self._angle))
            self._moduleStates.append(kinematics.SwerveModuleState(0, self._angle))

            self._modulePositions.append(
                kinematics.SwerveModulePosition(0, self._angle)
            )
            self._modulePositions.append(
                kinematics.SwerveModulePosition(0, self._angle)
            )
            self._modulePositions.append(
                kinematics.SwerveModulePosition(0, self._angle)
            )

            self._odometry = kinematics.SwerveDrive3Odometry(
                self._kinematics,
                self._angle,
                self._modulePositions,
                geometry.Pose2d(0, 0, geometry.Rotation2d(0)),
            )
        elif selector == FOUR:
            self._kinematics = kinematics.SwerveDrive4Kinematics(
                geometry.Translation2d(self._kWheelBase / 2, self._kTrackWidth / 2),
                geometry.Translation2d(self._kWheelBase / 2, -self._kTrackWidth / 2),
                geometry.Translation2d(self._kWheelBase / 2, self._kTrackWidth / 2),
                geometry.Translation2d(self._kWheelBase / 2, -self._kTrackWidth / 2),
            )

            self._moduleStates.append(kinematics.SwerveModuleState(0, self._angle))
            self._moduleStates.append(kinematics.SwerveModuleState(0, self._angle))
            self._moduleStates.append(kinematics.SwerveModuleState(0, self._angle))
            self._moduleStates.append(kinematics.SwerveModuleState(0, self._angle))

            self._modulePositions.append(
                kinematics.SwerveModulePosition(0, self._angle)
            )
            self._modulePositions.append(
                kinematics.SwerveModulePosition(0, self._angle)
            )
            self._modulePositions.append(
                kinematics.SwerveModulePosition(0, self._angle)
            )
            self._modulePositions.append(
                kinematics.SwerveModulePosition(0, self._angle)
            )

            self._odometry = kinematics.SwerveDrive4Odometry(
                self._kinematics,
                self._angle,
                self._modulePositions,
                geometry.Pose2d(0, 0, geometry.Rotation2d(0)),
            )
        elif selector == SIX:
            self._kinematics = kinematics.SwerveDrive6Kinematics(
                geometry.Translation2d(self._kWheelBase / 2, self._kTrackWidth / 2),
                geometry.Translation2d(self._kWheelBase / 2, -self._kTrackWidth / 2),
                geometry.Translation2d(self._kWheelBase / 2, self._kTrackWidth / 2),
                geometry.Translation2d(self._kWheelBase / 2, -self._kTrackWidth / 2),
                geometry.Translation2d(self._kWheelBase / 2, self._kTrackWidth / 2),
                geometry.Translation2d(self._kWheelBase / 2, -self._kTrackWidth / 2),
            )

            self._moduleStates.append(kinematics.SwerveModuleState(0, self._angle))
            self._moduleStates.append(kinematics.SwerveModuleState(0, self._angle))
            self._moduleStates.append(kinematics.SwerveModuleState(0, self._angle))
            self._moduleStates.append(kinematics.SwerveModuleState(0, self._angle))
            self._moduleStates.append(kinematics.SwerveModuleState(0, self._angle))
            self._moduleStates.append(kinematics.SwerveModuleState(0, self._angle))

            self._modulePositions.append(
                kinematics.SwerveModulePosition(0, self._angle)
            )
            self._modulePositions.append(
                kinematics.SwerveModulePosition(0, self._angle)
            )
            self._modulePositions.append(
                kinematics.SwerveModulePosition(0, self._angle)
            )
            self._modulePositions.append(
                kinematics.SwerveModulePosition(0, self._angle)
            )
            self._modulePositions.append(
                kinematics.SwerveModulePosition(0, self._angle)
            )
            self._modulePositions.append(
                kinematics.SwerveModulePosition(0, self._angle)
            )

            self._odometry = kinematics.SwerveDrive6Odometry(
                self._kinematics,
                self._angle,
                self._modulePositions,
                geometry.Pose2d(0, 0, geometry.Rotation2d(0)),
            )
        elif selector == MISMATCHED_KINEMATICS_AND_ODOMETRY:
            self._kinematics = kinematics.SwerveDrive2Kinematics(
                geometry.Translation2d(self._kWheelBase / 2, self._kTrackWidth / 2),
                geometry.Translation2d(self._kWheelBase / 2, -self._kTrackWidth / 2),
            )

            self._moduleStates.append(kinematics.SwerveModuleState(0, self._angle))
            self._moduleStates.append(kinematics.SwerveModuleState(0, self._angle))

            self._modulePositions.append(
                kinematics.SwerveModulePosition(0, self._angle)
            )
            self._modulePositions.append(
                kinematics.SwerveModulePosition(0, self._angle)
            )

            self._odometry = kinematics.SwerveDrive6Odometry(
                self._kinematics,
                self._angle,
                self._modulePositions,
                geometry.Pose2d(0, 0, geometry.Rotation2d(0)),
            )

    def setModuleStates(self, states: List[kinematics.SwerveModuleState]) -> None:
        self._moduleStates = states

    def getRobotPose(self) -> geometry.Pose2d:
        self._odometry.update(self._angle, self._modulePositions)

        return self._odometry.getPose()

    def getRotationHeadingZero(self) -> geometry.Rotation2d:
        return geometry.Rotation2d()


@pytest.fixture()
def get_swerve_controller_data() -> SwerveControllerCommandTestDataFixtures:
    def _method(selector):
        return SwerveControllerCommandTestDataFixtures(selector)

    return _method


def test_SwerveControllerMismatchedKinematicsAndOdometry(
    scheduler: commands2.CommandScheduler, get_swerve_controller_data
):
    with ManualSimTime() as sim:
        subsystem = commands2.Subsystem()
        waypoints: List[geometry.Pose2d] = []
        waypoints.append(geometry.Pose2d(0, 0, geometry.Rotation2d(0)))
        waypoints.append(geometry.Pose2d(1, 5, geometry.Rotation2d(3)))
        traj_config: trajectory.TrajectoryConfig = trajectory.TrajectoryConfig(8.8, 0.1)
        new_trajectory: trajectory.Trajectory = (
            trajectory.TrajectoryGenerator.generateTrajectory(waypoints, traj_config)
        )
        end_state = new_trajectory.sample(new_trajectory.totalTime())

        with pytest.raises(TypeError):
            fixture_data: SwerveControllerCommandTestDataFixtures = (
                get_swerve_controller_data(MISMATCHED_KINEMATICS_AND_ODOMETRY)
            )


def test_SwerveControllerIncompletePID(
    scheduler: commands2.CommandScheduler, get_swerve_controller_data
):
    with ManualSimTime() as sim:
        subsystem = commands2.Subsystem()
        waypoints: List[geometry.Pose2d] = []
        waypoints.append(geometry.Pose2d(0, 0, geometry.Rotation2d(0)))
        waypoints.append(geometry.Pose2d(1, 5, geometry.Rotation2d(3)))
        traj_config: trajectory.TrajectoryConfig = trajectory.TrajectoryConfig(8.8, 0.1)
        new_trajectory: trajectory.Trajectory = (
            trajectory.TrajectoryGenerator.generateTrajectory(waypoints, traj_config)
        )
        end_state = new_trajectory.sample(new_trajectory.totalTime())

        with pytest.raises(Exception):
            fixture_data: SwerveControllerCommandTestDataFixtures = (
                get_swerve_controller_data(INCOMPLETE_PID_CLASSES)
            )

            command = commands2.SwerveControllerCommand(
                new_trajectory,
                fixture_data.getRobotPose,
                fixture_data._kinematics,
                fixture_data.setModuleStates,
                fixture_data._xController,
                fixture_data._yController,
                fixture_data._rotationController,
                fixture_data.getRotationHeadingZero,
                subsystem,
            )


def test_SwerveControllerCommand2Holonomic(
    scheduler: commands2.CommandScheduler, get_swerve_controller_data
):
    with ManualSimTime() as sim:
        subsystem = commands2.Subsystem()
        waypoints: List[geometry.Pose2d] = []
        waypoints.append(geometry.Pose2d(0, 0, geometry.Rotation2d(0)))
        waypoints.append(geometry.Pose2d(1, 5, geometry.Rotation2d(3)))
        traj_config: trajectory.TrajectoryConfig = trajectory.TrajectoryConfig(8.8, 0.1)
        new_trajectory: trajectory.Trajectory = (
            trajectory.TrajectoryGenerator.generateTrajectory(waypoints, traj_config)
        )
        end_state = new_trajectory.sample(new_trajectory.totalTime())

        fixture_data: SwerveControllerCommandTestDataFixtures = (
            get_swerve_controller_data(TWO)
        )

        command = commands2.SwerveControllerCommand(
            new_trajectory,
            fixture_data.getRobotPose,
            fixture_data._kinematics,
            fixture_data._holonomic,
            fixture_data.setModuleStates,
            subsystem,
        )

        fixture_data._timer.restart()

        command.initialize()

        while not command.isFinished():
            command.execute()
            fixture_data._angle = new_trajectory.sample(
                fixture_data._timer.get()
            ).pose.rotation()

            for i in range(0, len(fixture_data._modulePositions)):
                fixture_data._modulePositions[i].distance += (
                    fixture_data._moduleStates[i].speed * 0.005
                )
                fixture_data._modulePositions[i].angle = fixture_data._moduleStates[
                    i
                ].angle

            sim.step(0.005)

        fixture_data._timer.stop()
        command.end(True)

        assert end_state.pose.X() == pytest.approx(
            fixture_data.getRobotPose().X(), fixture_data._kxTolerance
        )
        assert end_state.pose.Y() == pytest.approx(
            fixture_data.getRobotPose().Y(), fixture_data._kyTolerance
        )
        assert end_state.pose.rotation().radians() == pytest.approx(
            fixture_data.getRobotPose().rotation().radians(),
            fixture_data._kAngularTolerance,
        )


def test_SwerveControllerCommand2HolonomicWithRotation(
    scheduler: commands2.CommandScheduler, get_swerve_controller_data
):
    with ManualSimTime() as sim:
        subsystem = commands2.Subsystem()
        waypoints: List[geometry.Pose2d] = []
        waypoints.append(geometry.Pose2d(0, 0, geometry.Rotation2d(0)))
        waypoints.append(geometry.Pose2d(1, 5, geometry.Rotation2d(3)))
        traj_config: trajectory.TrajectoryConfig = trajectory.TrajectoryConfig(8.8, 0.1)
        new_trajectory: trajectory.Trajectory = (
            trajectory.TrajectoryGenerator.generateTrajectory(waypoints, traj_config)
        )
        end_state = new_trajectory.sample(new_trajectory.totalTime())

        fixture_data: SwerveControllerCommandTestDataFixtures = (
            get_swerve_controller_data(TWO)
        )

        command = commands2.SwerveControllerCommand(
            new_trajectory,
            fixture_data.getRobotPose,
            fixture_data._kinematics,
            fixture_data._holonomic,
            fixture_data.getRotationHeadingZero,
            fixture_data.setModuleStates,
            subsystem,
        )

        fixture_data._timer.restart()

        command.initialize()

        while not command.isFinished():
            command.execute()
            fixture_data._angle = new_trajectory.sample(
                fixture_data._timer.get()
            ).pose.rotation()

            for i in range(0, len(fixture_data._modulePositions)):
                fixture_data._modulePositions[i].distance += (
                    fixture_data._moduleStates[i].speed * 0.005
                )
                fixture_data._modulePositions[i].angle = fixture_data._moduleStates[
                    i
                ].angle

            sim.step(0.005)

        fixture_data._timer.stop()
        command.end(True)

        assert end_state.pose.X() == pytest.approx(
            fixture_data.getRobotPose().X(), fixture_data._kxTolerance
        )
        assert end_state.pose.Y() == pytest.approx(
            fixture_data.getRobotPose().Y(), fixture_data._kyTolerance
        )
        assert end_state.pose.rotation().radians() == pytest.approx(
            fixture_data.getRobotPose().rotation().radians(),
            fixture_data._kAngularTolerance,
        )


def test_SwerveControllerCommand2PID(
    scheduler: commands2.CommandScheduler, get_swerve_controller_data
):
    with ManualSimTime() as sim:
        subsystem = commands2.Subsystem()
        waypoints: List[geometry.Pose2d] = []
        waypoints.append(geometry.Pose2d(0, 0, geometry.Rotation2d(0)))
        waypoints.append(geometry.Pose2d(1, 5, geometry.Rotation2d(3)))
        traj_config: trajectory.TrajectoryConfig = trajectory.TrajectoryConfig(8.8, 0.1)
        new_trajectory: trajectory.Trajectory = (
            trajectory.TrajectoryGenerator.generateTrajectory(waypoints, traj_config)
        )
        end_state = new_trajectory.sample(new_trajectory.totalTime())

        fixture_data: SwerveControllerCommandTestDataFixtures = (
            get_swerve_controller_data(TWO)
        )

        command = commands2.SwerveControllerCommand(
            new_trajectory,
            fixture_data.getRobotPose,
            fixture_data._kinematics,
            fixture_data._xController,
            fixture_data._yController,
            fixture_data._rotationController,
            fixture_data.setModuleStates,
            subsystem,
        )

        fixture_data._timer.restart()

        command.initialize()

        while not command.isFinished():
            command.execute()
            fixture_data._angle = new_trajectory.sample(
                fixture_data._timer.get()
            ).pose.rotation()

            for i in range(0, len(fixture_data._modulePositions)):
                fixture_data._modulePositions[i].distance += (
                    fixture_data._moduleStates[i].speed * 0.005
                )
                fixture_data._modulePositions[i].angle = fixture_data._moduleStates[
                    i
                ].angle

            sim.step(0.005)

        fixture_data._timer.stop()
        command.end(True)

        assert end_state.pose.X() == pytest.approx(
            fixture_data.getRobotPose().X(), fixture_data._kxTolerance
        )
        assert end_state.pose.Y() == pytest.approx(
            fixture_data.getRobotPose().Y(), fixture_data._kyTolerance
        )
        assert end_state.pose.rotation().radians() == pytest.approx(
            fixture_data.getRobotPose().rotation().radians(),
            fixture_data._kAngularTolerance,
        )


def test_SwerveControllerCommand2PIDWithRotation(
    scheduler: commands2.CommandScheduler, get_swerve_controller_data
):
    with ManualSimTime() as sim:
        subsystem = commands2.Subsystem()
        waypoints: List[geometry.Pose2d] = []
        waypoints.append(geometry.Pose2d(0, 0, geometry.Rotation2d(0)))
        waypoints.append(geometry.Pose2d(1, 5, geometry.Rotation2d(3)))
        traj_config: trajectory.TrajectoryConfig = trajectory.TrajectoryConfig(8.8, 0.1)
        new_trajectory: trajectory.Trajectory = (
            trajectory.TrajectoryGenerator.generateTrajectory(waypoints, traj_config)
        )
        end_state = new_trajectory.sample(new_trajectory.totalTime())

        fixture_data: SwerveControllerCommandTestDataFixtures = (
            get_swerve_controller_data(TWO)
        )

        command = commands2.SwerveControllerCommand(
            new_trajectory,
            fixture_data.getRobotPose,
            fixture_data._kinematics,
            fixture_data._xController,
            fixture_data._yController,
            fixture_data._rotationController,
            fixture_data.getRotationHeadingZero,
            fixture_data.setModuleStates,
            subsystem,
        )

        fixture_data._timer.restart()

        command.initialize()

        while not command.isFinished():
            command.execute()
            fixture_data._angle = new_trajectory.sample(
                fixture_data._timer.get()
            ).pose.rotation()

            for i in range(0, len(fixture_data._modulePositions)):
                fixture_data._modulePositions[i].distance += (
                    fixture_data._moduleStates[i].speed * 0.005
                )
                fixture_data._modulePositions[i].angle = fixture_data._moduleStates[
                    i
                ].angle

            sim.step(0.005)

        fixture_data._timer.stop()
        command.end(True)

        assert end_state.pose.X() == pytest.approx(
            fixture_data.getRobotPose().X(), fixture_data._kxTolerance
        )
        assert end_state.pose.Y() == pytest.approx(
            fixture_data.getRobotPose().Y(), fixture_data._kyTolerance
        )
        assert end_state.pose.rotation().radians() == pytest.approx(
            fixture_data.getRobotPose().rotation().radians(),
            fixture_data._kAngularTolerance,
        )


def test_SwerveControllerCommand3Holonomic(
    scheduler: commands2.CommandScheduler, get_swerve_controller_data
):
    with ManualSimTime() as sim:
        subsystem = commands2.Subsystem()
        waypoints: List[geometry.Pose2d] = []
        waypoints.append(geometry.Pose2d(0, 0, geometry.Rotation2d(0)))
        waypoints.append(geometry.Pose2d(1, 5, geometry.Rotation2d(3)))
        traj_config: trajectory.TrajectoryConfig = trajectory.TrajectoryConfig(8.8, 0.1)
        new_trajectory: trajectory.Trajectory = (
            trajectory.TrajectoryGenerator.generateTrajectory(waypoints, traj_config)
        )
        end_state = new_trajectory.sample(new_trajectory.totalTime())

        fixture_data: SwerveControllerCommandTestDataFixtures = (
            get_swerve_controller_data(THREE)
        )

        command = commands2.SwerveControllerCommand(
            new_trajectory,
            fixture_data.getRobotPose,
            fixture_data._kinematics,
            fixture_data._holonomic,
            fixture_data.setModuleStates,
            subsystem,
        )

        fixture_data._timer.restart()

        command.initialize()

        while not command.isFinished():
            command.execute()
            fixture_data._angle = new_trajectory.sample(
                fixture_data._timer.get()
            ).pose.rotation()

            for i in range(0, len(fixture_data._modulePositions)):
                fixture_data._modulePositions[i].distance += (
                    fixture_data._moduleStates[i].speed * 0.005
                )
                fixture_data._modulePositions[i].angle = fixture_data._moduleStates[
                    i
                ].angle

            sim.step(0.005)

        fixture_data._timer.stop()
        command.end(True)

        assert end_state.pose.X() == pytest.approx(
            fixture_data.getRobotPose().X(), fixture_data._kxTolerance
        )
        assert end_state.pose.Y() == pytest.approx(
            fixture_data.getRobotPose().Y(), fixture_data._kyTolerance
        )
        assert end_state.pose.rotation().radians() == pytest.approx(
            fixture_data.getRobotPose().rotation().radians(),
            fixture_data._kAngularTolerance,
        )


def test_SwerveControllerCommand3HolonomicWithRotation(
    scheduler: commands2.CommandScheduler, get_swerve_controller_data
):
    with ManualSimTime() as sim:
        subsystem = commands2.Subsystem()
        waypoints: List[geometry.Pose2d] = []
        waypoints.append(geometry.Pose2d(0, 0, geometry.Rotation2d(0)))
        waypoints.append(geometry.Pose2d(1, 5, geometry.Rotation2d(3)))
        traj_config: trajectory.TrajectoryConfig = trajectory.TrajectoryConfig(8.8, 0.1)
        new_trajectory: trajectory.Trajectory = (
            trajectory.TrajectoryGenerator.generateTrajectory(waypoints, traj_config)
        )
        end_state = new_trajectory.sample(new_trajectory.totalTime())

        fixture_data: SwerveControllerCommandTestDataFixtures = (
            get_swerve_controller_data(THREE)
        )

        command = commands2.SwerveControllerCommand(
            new_trajectory,
            fixture_data.getRobotPose,
            fixture_data._kinematics,
            fixture_data._holonomic,
            fixture_data.getRotationHeadingZero,
            fixture_data.setModuleStates,
            subsystem,
        )

        fixture_data._timer.restart()

        command.initialize()

        while not command.isFinished():
            command.execute()
            fixture_data._angle = new_trajectory.sample(
                fixture_data._timer.get()
            ).pose.rotation()

            for i in range(0, len(fixture_data._modulePositions)):
                fixture_data._modulePositions[i].distance += (
                    fixture_data._moduleStates[i].speed * 0.005
                )
                fixture_data._modulePositions[i].angle = fixture_data._moduleStates[
                    i
                ].angle

            sim.step(0.005)

        fixture_data._timer.stop()
        command.end(True)

        assert end_state.pose.X() == pytest.approx(
            fixture_data.getRobotPose().X(), fixture_data._kxTolerance
        )
        assert end_state.pose.Y() == pytest.approx(
            fixture_data.getRobotPose().Y(), fixture_data._kyTolerance
        )
        assert end_state.pose.rotation().radians() == pytest.approx(
            fixture_data.getRobotPose().rotation().radians(),
            fixture_data._kAngularTolerance,
        )


def test_SwerveControllerCommand3PID(
    scheduler: commands2.CommandScheduler, get_swerve_controller_data
):
    with ManualSimTime() as sim:
        subsystem = commands2.Subsystem()
        waypoints: List[geometry.Pose2d] = []
        waypoints.append(geometry.Pose2d(0, 0, geometry.Rotation2d(0)))
        waypoints.append(geometry.Pose2d(1, 5, geometry.Rotation2d(3)))
        traj_config: trajectory.TrajectoryConfig = trajectory.TrajectoryConfig(8.8, 0.1)
        new_trajectory: trajectory.Trajectory = (
            trajectory.TrajectoryGenerator.generateTrajectory(waypoints, traj_config)
        )
        end_state = new_trajectory.sample(new_trajectory.totalTime())

        fixture_data: SwerveControllerCommandTestDataFixtures = (
            get_swerve_controller_data(THREE)
        )

        command = commands2.SwerveControllerCommand(
            new_trajectory,
            fixture_data.getRobotPose,
            fixture_data._kinematics,
            fixture_data._xController,
            fixture_data._yController,
            fixture_data._rotationController,
            fixture_data.setModuleStates,
            subsystem,
        )

        fixture_data._timer.restart()

        command.initialize()

        while not command.isFinished():
            command.execute()
            fixture_data._angle = new_trajectory.sample(
                fixture_data._timer.get()
            ).pose.rotation()

            for i in range(0, len(fixture_data._modulePositions)):
                fixture_data._modulePositions[i].distance += (
                    fixture_data._moduleStates[i].speed * 0.005
                )
                fixture_data._modulePositions[i].angle = fixture_data._moduleStates[
                    i
                ].angle

            sim.step(0.005)

        fixture_data._timer.stop()
        command.end(True)

        assert end_state.pose.X() == pytest.approx(
            fixture_data.getRobotPose().X(), fixture_data._kxTolerance
        )
        assert end_state.pose.Y() == pytest.approx(
            fixture_data.getRobotPose().Y(), fixture_data._kyTolerance
        )
        assert end_state.pose.rotation().radians() == pytest.approx(
            fixture_data.getRobotPose().rotation().radians(),
            fixture_data._kAngularTolerance,
        )


def test_SwerveControllerCommand3PIDWithRotation(
    scheduler: commands2.CommandScheduler, get_swerve_controller_data
):
    with ManualSimTime() as sim:
        subsystem = commands2.Subsystem()
        waypoints: List[geometry.Pose2d] = []
        waypoints.append(geometry.Pose2d(0, 0, geometry.Rotation2d(0)))
        waypoints.append(geometry.Pose2d(1, 5, geometry.Rotation2d(3)))
        traj_config: trajectory.TrajectoryConfig = trajectory.TrajectoryConfig(8.8, 0.1)
        new_trajectory: trajectory.Trajectory = (
            trajectory.TrajectoryGenerator.generateTrajectory(waypoints, traj_config)
        )
        end_state = new_trajectory.sample(new_trajectory.totalTime())

        fixture_data: SwerveControllerCommandTestDataFixtures = (
            get_swerve_controller_data(THREE)
        )

        command = commands2.SwerveControllerCommand(
            new_trajectory,
            fixture_data.getRobotPose,
            fixture_data._kinematics,
            fixture_data._xController,
            fixture_data._yController,
            fixture_data._rotationController,
            fixture_data.getRotationHeadingZero,
            fixture_data.setModuleStates,
            subsystem,
        )

        fixture_data._timer.restart()

        command.initialize()

        while not command.isFinished():
            command.execute()
            fixture_data._angle = new_trajectory.sample(
                fixture_data._timer.get()
            ).pose.rotation()

            for i in range(0, len(fixture_data._modulePositions)):
                fixture_data._modulePositions[i].distance += (
                    fixture_data._moduleStates[i].speed * 0.005
                )
                fixture_data._modulePositions[i].angle = fixture_data._moduleStates[
                    i
                ].angle

            sim.step(0.005)

        fixture_data._timer.stop()
        command.end(True)

        assert end_state.pose.X() == pytest.approx(
            fixture_data.getRobotPose().X(), fixture_data._kxTolerance
        )
        assert end_state.pose.Y() == pytest.approx(
            fixture_data.getRobotPose().Y(), fixture_data._kyTolerance
        )
        assert end_state.pose.rotation().radians() == pytest.approx(
            fixture_data.getRobotPose().rotation().radians(),
            fixture_data._kAngularTolerance,
        )


def test_SwerveControllerCommand4Holonomic(
    scheduler: commands2.CommandScheduler, get_swerve_controller_data
):
    with ManualSimTime() as sim:
        subsystem = commands2.Subsystem()
        waypoints: List[geometry.Pose2d] = []
        waypoints.append(geometry.Pose2d(0, 0, geometry.Rotation2d(0)))
        waypoints.append(geometry.Pose2d(1, 5, geometry.Rotation2d(3)))
        traj_config: trajectory.TrajectoryConfig = trajectory.TrajectoryConfig(8.8, 0.1)
        new_trajectory: trajectory.Trajectory = (
            trajectory.TrajectoryGenerator.generateTrajectory(waypoints, traj_config)
        )
        end_state = new_trajectory.sample(new_trajectory.totalTime())

        fixture_data: SwerveControllerCommandTestDataFixtures = (
            get_swerve_controller_data(FOUR)
        )

        command = commands2.SwerveControllerCommand(
            new_trajectory,
            fixture_data.getRobotPose,
            fixture_data._kinematics,
            fixture_data._holonomic,
            fixture_data.setModuleStates,
            subsystem,
        )

        fixture_data._timer.restart()

        command.initialize()

        while not command.isFinished():
            command.execute()
            fixture_data._angle = new_trajectory.sample(
                fixture_data._timer.get()
            ).pose.rotation()

            for i in range(0, len(fixture_data._modulePositions)):
                fixture_data._modulePositions[i].distance += (
                    fixture_data._moduleStates[i].speed * 0.005
                )
                fixture_data._modulePositions[i].angle = fixture_data._moduleStates[
                    i
                ].angle

            sim.step(0.005)

        fixture_data._timer.stop()
        command.end(True)

        assert end_state.pose.X() == pytest.approx(
            fixture_data.getRobotPose().X(), fixture_data._kxTolerance
        )
        assert end_state.pose.Y() == pytest.approx(
            fixture_data.getRobotPose().Y(), fixture_data._kyTolerance
        )
        assert end_state.pose.rotation().radians() == pytest.approx(
            fixture_data.getRobotPose().rotation().radians(),
            fixture_data._kAngularTolerance,
        )


def test_SwerveControllerCommand4HolonomicWithRotation(
    scheduler: commands2.CommandScheduler, get_swerve_controller_data
):
    with ManualSimTime() as sim:
        subsystem = commands2.Subsystem()
        waypoints: List[geometry.Pose2d] = []
        waypoints.append(geometry.Pose2d(0, 0, geometry.Rotation2d(0)))
        waypoints.append(geometry.Pose2d(1, 5, geometry.Rotation2d(3)))
        traj_config: trajectory.TrajectoryConfig = trajectory.TrajectoryConfig(8.8, 0.1)
        new_trajectory: trajectory.Trajectory = (
            trajectory.TrajectoryGenerator.generateTrajectory(waypoints, traj_config)
        )
        end_state = new_trajectory.sample(new_trajectory.totalTime())

        fixture_data: SwerveControllerCommandTestDataFixtures = (
            get_swerve_controller_data(FOUR)
        )

        command = commands2.SwerveControllerCommand(
            new_trajectory,
            fixture_data.getRobotPose,
            fixture_data._kinematics,
            fixture_data._holonomic,
            fixture_data.getRotationHeadingZero,
            fixture_data.setModuleStates,
            subsystem,
        )

        fixture_data._timer.restart()

        command.initialize()

        while not command.isFinished():
            command.execute()
            fixture_data._angle = new_trajectory.sample(
                fixture_data._timer.get()
            ).pose.rotation()

            for i in range(0, len(fixture_data._modulePositions)):
                fixture_data._modulePositions[i].distance += (
                    fixture_data._moduleStates[i].speed * 0.005
                )
                fixture_data._modulePositions[i].angle = fixture_data._moduleStates[
                    i
                ].angle

            sim.step(0.005)

        fixture_data._timer.stop()
        command.end(True)

        assert end_state.pose.X() == pytest.approx(
            fixture_data.getRobotPose().X(), fixture_data._kxTolerance
        )
        assert end_state.pose.Y() == pytest.approx(
            fixture_data.getRobotPose().Y(), fixture_data._kyTolerance
        )
        assert end_state.pose.rotation().radians() == pytest.approx(
            fixture_data.getRobotPose().rotation().radians(),
            fixture_data._kAngularTolerance,
        )


def test_SwerveControllerCommand4PID(
    scheduler: commands2.CommandScheduler, get_swerve_controller_data
):
    with ManualSimTime() as sim:
        subsystem = commands2.Subsystem()
        waypoints: List[geometry.Pose2d] = []
        waypoints.append(geometry.Pose2d(0, 0, geometry.Rotation2d(0)))
        waypoints.append(geometry.Pose2d(1, 5, geometry.Rotation2d(3)))
        traj_config: trajectory.TrajectoryConfig = trajectory.TrajectoryConfig(8.8, 0.1)
        new_trajectory: trajectory.Trajectory = (
            trajectory.TrajectoryGenerator.generateTrajectory(waypoints, traj_config)
        )
        end_state = new_trajectory.sample(new_trajectory.totalTime())

        fixture_data: SwerveControllerCommandTestDataFixtures = (
            get_swerve_controller_data(FOUR)
        )

        command = commands2.SwerveControllerCommand(
            new_trajectory,
            fixture_data.getRobotPose,
            fixture_data._kinematics,
            fixture_data._xController,
            fixture_data._yController,
            fixture_data._rotationController,
            fixture_data.setModuleStates,
            subsystem,
        )

        fixture_data._timer.restart()

        command.initialize()

        while not command.isFinished():
            command.execute()
            fixture_data._angle = new_trajectory.sample(
                fixture_data._timer.get()
            ).pose.rotation()

            for i in range(0, len(fixture_data._modulePositions)):
                fixture_data._modulePositions[i].distance += (
                    fixture_data._moduleStates[i].speed * 0.005
                )
                fixture_data._modulePositions[i].angle = fixture_data._moduleStates[
                    i
                ].angle

            sim.step(0.005)

        fixture_data._timer.stop()
        command.end(True)

        assert end_state.pose.X() == pytest.approx(
            fixture_data.getRobotPose().X(), fixture_data._kxTolerance
        )
        assert end_state.pose.Y() == pytest.approx(
            fixture_data.getRobotPose().Y(), fixture_data._kyTolerance
        )
        assert end_state.pose.rotation().radians() == pytest.approx(
            fixture_data.getRobotPose().rotation().radians(),
            fixture_data._kAngularTolerance,
        )


def test_SwerveControllerCommand4PIDWithRotation(
    scheduler: commands2.CommandScheduler, get_swerve_controller_data
):
    with ManualSimTime() as sim:
        subsystem = commands2.Subsystem()
        waypoints: List[geometry.Pose2d] = []
        waypoints.append(geometry.Pose2d(0, 0, geometry.Rotation2d(0)))
        waypoints.append(geometry.Pose2d(1, 5, geometry.Rotation2d(3)))
        traj_config: trajectory.TrajectoryConfig = trajectory.TrajectoryConfig(8.8, 0.1)
        new_trajectory: trajectory.Trajectory = (
            trajectory.TrajectoryGenerator.generateTrajectory(waypoints, traj_config)
        )
        end_state = new_trajectory.sample(new_trajectory.totalTime())

        fixture_data: SwerveControllerCommandTestDataFixtures = (
            get_swerve_controller_data(FOUR)
        )

        command = commands2.SwerveControllerCommand(
            new_trajectory,
            fixture_data.getRobotPose,
            fixture_data._kinematics,
            fixture_data._xController,
            fixture_data._yController,
            fixture_data._rotationController,
            fixture_data.getRotationHeadingZero,
            fixture_data.setModuleStates,
            subsystem,
        )

        fixture_data._timer.restart()

        command.initialize()

        while not command.isFinished():
            command.execute()
            fixture_data._angle = new_trajectory.sample(
                fixture_data._timer.get()
            ).pose.rotation()

            for i in range(0, len(fixture_data._modulePositions)):
                fixture_data._modulePositions[i].distance += (
                    fixture_data._moduleStates[i].speed * 0.005
                )
                fixture_data._modulePositions[i].angle = fixture_data._moduleStates[
                    i
                ].angle

            sim.step(0.005)

        fixture_data._timer.stop()
        command.end(True)

        assert end_state.pose.X() == pytest.approx(
            fixture_data.getRobotPose().X(), fixture_data._kxTolerance
        )
        assert end_state.pose.Y() == pytest.approx(
            fixture_data.getRobotPose().Y(), fixture_data._kyTolerance
        )
        assert end_state.pose.rotation().radians() == pytest.approx(
            fixture_data.getRobotPose().rotation().radians(),
            fixture_data._kAngularTolerance,
        )


def test_SwerveControllerCommand6Holonomic(
    scheduler: commands2.CommandScheduler, get_swerve_controller_data
):
    with ManualSimTime() as sim:
        subsystem = commands2.Subsystem()
        waypoints: List[geometry.Pose2d] = []
        waypoints.append(geometry.Pose2d(0, 0, geometry.Rotation2d(0)))
        waypoints.append(geometry.Pose2d(1, 5, geometry.Rotation2d(3)))
        traj_config: trajectory.TrajectoryConfig = trajectory.TrajectoryConfig(8.8, 0.1)
        new_trajectory: trajectory.Trajectory = (
            trajectory.TrajectoryGenerator.generateTrajectory(waypoints, traj_config)
        )
        end_state = new_trajectory.sample(new_trajectory.totalTime())

        fixture_data: SwerveControllerCommandTestDataFixtures = (
            get_swerve_controller_data(SIX)
        )

        command = commands2.SwerveControllerCommand(
            new_trajectory,
            fixture_data.getRobotPose,
            fixture_data._kinematics,
            fixture_data._holonomic,
            fixture_data.setModuleStates,
            subsystem,
        )

        fixture_data._timer.restart()

        command.initialize()

        while not command.isFinished():
            command.execute()
            fixture_data._angle = new_trajectory.sample(
                fixture_data._timer.get()
            ).pose.rotation()

            for i in range(0, len(fixture_data._modulePositions)):
                fixture_data._modulePositions[i].distance += (
                    fixture_data._moduleStates[i].speed * 0.005
                )
                fixture_data._modulePositions[i].angle = fixture_data._moduleStates[
                    i
                ].angle

            sim.step(0.005)

        fixture_data._timer.stop()
        command.end(True)

        assert end_state.pose.X() == pytest.approx(
            fixture_data.getRobotPose().X(), fixture_data._kxTolerance
        )
        assert end_state.pose.Y() == pytest.approx(
            fixture_data.getRobotPose().Y(), fixture_data._kyTolerance
        )
        assert end_state.pose.rotation().radians() == pytest.approx(
            fixture_data.getRobotPose().rotation().radians(),
            fixture_data._kAngularTolerance,
        )


def test_SwerveControllerCommand6HolonomicWithRotation(
    scheduler: commands2.CommandScheduler, get_swerve_controller_data
):
    with ManualSimTime() as sim:
        subsystem = commands2.Subsystem()
        waypoints: List[geometry.Pose2d] = []
        waypoints.append(geometry.Pose2d(0, 0, geometry.Rotation2d(0)))
        waypoints.append(geometry.Pose2d(1, 5, geometry.Rotation2d(3)))
        traj_config: trajectory.TrajectoryConfig = trajectory.TrajectoryConfig(8.8, 0.1)
        new_trajectory: trajectory.Trajectory = (
            trajectory.TrajectoryGenerator.generateTrajectory(waypoints, traj_config)
        )
        end_state = new_trajectory.sample(new_trajectory.totalTime())

        fixture_data: SwerveControllerCommandTestDataFixtures = (
            get_swerve_controller_data(SIX)
        )

        command = commands2.SwerveControllerCommand(
            new_trajectory,
            fixture_data.getRobotPose,
            fixture_data._kinematics,
            fixture_data._holonomic,
            fixture_data.getRotationHeadingZero,
            fixture_data.setModuleStates,
            subsystem,
        )

        fixture_data._timer.restart()

        command.initialize()

        while not command.isFinished():
            command.execute()
            fixture_data._angle = new_trajectory.sample(
                fixture_data._timer.get()
            ).pose.rotation()

            for i in range(0, len(fixture_data._modulePositions)):
                fixture_data._modulePositions[i].distance += (
                    fixture_data._moduleStates[i].speed * 0.005
                )
                fixture_data._modulePositions[i].angle = fixture_data._moduleStates[
                    i
                ].angle

            sim.step(0.005)

        fixture_data._timer.stop()
        command.end(True)

        assert end_state.pose.X() == pytest.approx(
            fixture_data.getRobotPose().X(), fixture_data._kxTolerance
        )
        assert end_state.pose.Y() == pytest.approx(
            fixture_data.getRobotPose().Y(), fixture_data._kyTolerance
        )
        assert end_state.pose.rotation().radians() == pytest.approx(
            fixture_data.getRobotPose().rotation().radians(),
            fixture_data._kAngularTolerance,
        )


def test_SwerveControllerCommand6PID(
    scheduler: commands2.CommandScheduler, get_swerve_controller_data
):
    with ManualSimTime() as sim:
        subsystem = commands2.Subsystem()
        waypoints: List[geometry.Pose2d] = []
        waypoints.append(geometry.Pose2d(0, 0, geometry.Rotation2d(0)))
        waypoints.append(geometry.Pose2d(1, 5, geometry.Rotation2d(3)))
        traj_config: trajectory.TrajectoryConfig = trajectory.TrajectoryConfig(8.8, 0.1)
        new_trajectory: trajectory.Trajectory = (
            trajectory.TrajectoryGenerator.generateTrajectory(waypoints, traj_config)
        )
        end_state = new_trajectory.sample(new_trajectory.totalTime())

        fixture_data: SwerveControllerCommandTestDataFixtures = (
            get_swerve_controller_data(SIX)
        )

        command = commands2.SwerveControllerCommand(
            new_trajectory,
            fixture_data.getRobotPose,
            fixture_data._kinematics,
            fixture_data._xController,
            fixture_data._yController,
            fixture_data._rotationController,
            fixture_data.setModuleStates,
            subsystem,
        )

        fixture_data._timer.restart()

        command.initialize()

        while not command.isFinished():
            command.execute()
            fixture_data._angle = new_trajectory.sample(
                fixture_data._timer.get()
            ).pose.rotation()

            for i in range(0, len(fixture_data._modulePositions)):
                fixture_data._modulePositions[i].distance += (
                    fixture_data._moduleStates[i].speed * 0.005
                )
                fixture_data._modulePositions[i].angle = fixture_data._moduleStates[
                    i
                ].angle

            sim.step(0.005)

        fixture_data._timer.stop()
        command.end(True)

        assert end_state.pose.X() == pytest.approx(
            fixture_data.getRobotPose().X(), fixture_data._kxTolerance
        )
        assert end_state.pose.Y() == pytest.approx(
            fixture_data.getRobotPose().Y(), fixture_data._kyTolerance
        )
        assert end_state.pose.rotation().radians() == pytest.approx(
            fixture_data.getRobotPose().rotation().radians(),
            fixture_data._kAngularTolerance,
        )


def test_SwerveControllerCommand6PIDWithRotation(
    scheduler: commands2.CommandScheduler, get_swerve_controller_data
):
    with ManualSimTime() as sim:
        subsystem = commands2.Subsystem()
        waypoints: List[geometry.Pose2d] = []
        waypoints.append(geometry.Pose2d(0, 0, geometry.Rotation2d(0)))
        waypoints.append(geometry.Pose2d(1, 5, geometry.Rotation2d(3)))
        traj_config: trajectory.TrajectoryConfig = trajectory.TrajectoryConfig(8.8, 0.1)
        new_trajectory: trajectory.Trajectory = (
            trajectory.TrajectoryGenerator.generateTrajectory(waypoints, traj_config)
        )
        end_state = new_trajectory.sample(new_trajectory.totalTime())

        fixture_data: SwerveControllerCommandTestDataFixtures = (
            get_swerve_controller_data(SIX)
        )

        command = commands2.SwerveControllerCommand(
            new_trajectory,
            fixture_data.getRobotPose,
            fixture_data._kinematics,
            fixture_data._xController,
            fixture_data._yController,
            fixture_data._rotationController,
            fixture_data.getRotationHeadingZero,
            fixture_data.setModuleStates,
            subsystem,
        )

        fixture_data._timer.restart()

        command.initialize()

        while not command.isFinished():
            command.execute()
            fixture_data._angle = new_trajectory.sample(
                fixture_data._timer.get()
            ).pose.rotation()

            for i in range(0, len(fixture_data._modulePositions)):
                fixture_data._modulePositions[i].distance += (
                    fixture_data._moduleStates[i].speed * 0.005
                )
                fixture_data._modulePositions[i].angle = fixture_data._moduleStates[
                    i
                ].angle

            sim.step(0.005)

        fixture_data._timer.stop()
        command.end(True)

        assert end_state.pose.X() == pytest.approx(
            fixture_data.getRobotPose().X(), fixture_data._kxTolerance
        )
        assert end_state.pose.Y() == pytest.approx(
            fixture_data.getRobotPose().Y(), fixture_data._kyTolerance
        )
        assert end_state.pose.rotation().radians() == pytest.approx(
            fixture_data.getRobotPose().rotation().radians(),
            fixture_data._kAngularTolerance,
        )
