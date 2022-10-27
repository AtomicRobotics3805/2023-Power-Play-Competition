/*
    Copyright (c) 2022 Atomic Robotics (https://atomicrobotics3805.org)

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see https://www.gnu.org/licenses/.
*/
package org.firstinspires.ftc.teamcode.trajectoryFactory

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.atomicrobotics.cflib.Constants
import com.atomicrobotics.cflib.trajectories.*


/**
 * This class contains all of the RoadRunner trajectories and start positions in the project. It's
 * used by the ExampleRoutines class. You can find how to use each of the possible trajectory
 * segments (like back and splineToSplineHeading) here:
 * https://learnroadrunner.com/trajectorybuilder-functions.html
 */
public object CompetitionTrajectoryFactory : TrajectoryFactory() {

    // start position declarations
    lateinit var startPoseAway: Pose2d

    // trajectory declarations
    lateinit var awayStartToLowJunction: ParallelTrajectory
    lateinit var awayLowJunctionToTerminal: ParallelTrajectory
    lateinit var awayLowJunctionToAwaySignalBlue: ParallelTrajectory
    lateinit var awayLowJunctionToAwaySignalRed: ParallelTrajectory

    // Signal results
    lateinit var e5SignalResultRed: ParallelTrajectory // Left
    lateinit var e5SignalResultGreen: ParallelTrajectory // Center
    lateinit var awaySignalResultBlue: ParallelTrajectory // Right

    /**
     * Initializes the robot's start positions and trajectories. This is where the trajectories are
     * actually created.
     */
    override fun initialize() {
        super.initialize()
        // start positions
        startPoseAway = Pose2d(30.0, 62.75.switchColor, 270.0.switchColorAngle.toRadians)

        // trajectories
        awayStartToLowJunction = Constants.drive.trajectoryBuilder(startPoseAway)
            .lineTo(Vector2d(23.8, 52.5.switchColor))
            .build()
        awayLowJunctionToTerminal = Constants.drive.trajectoryBuilder(awayStartToLowJunction.end())
            .lineTo(Vector2d(12.3, 59.5.switchColor))
            .build()
        awayLowJunctionToAwaySignalBlue = Constants.drive.trajectoryBuilder(awayStartToLowJunction.end())
            .lineTo(Vector2d(23.8, 56.2.switchColor))
            .splineToConstantHeading(Vector2d(32.0, 60.0.switchColor), 300.0.switchColorAngle.toRadians)
            .splineToSplineHeading(Pose2d(33.8, 47.2.switchColor, 250.0.switchColorAngle.toRadians), 250.0.switchColorAngle.toRadians)
            .build()
        awayLowJunctionToAwaySignalRed = Constants.drive.trajectoryBuilder(awayStartToLowJunction.end())
            .lineTo(Vector2d(23.8, 56.2.switchColor))
            .splineToConstantHeading(Vector2d(32.0, 60.0.switchColor), 300.0.switchColorAngle.toRadians)
            .splineToSplineHeading(Pose2d(33.8, 47.2.switchColor, 300.0.switchColorAngle.toRadians), 250.0.switchColorAngle.toRadians)
            .build()

        awaySignalResultBlue = Constants.drive.trajectoryBuilder(if(Constants.color == Constants.Color.RED) awayLowJunctionToAwaySignalRed.end() else awayLowJunctionToAwaySignalBlue.end())
            .splineToSplineHeading(Pose2d(11.3.flipAlongX36, 34.8.switchColor, 90.0.switchColorAngle.toRadians), 180.0.switchApproachTangentAngle.toRadians)
            .build()
    }
}