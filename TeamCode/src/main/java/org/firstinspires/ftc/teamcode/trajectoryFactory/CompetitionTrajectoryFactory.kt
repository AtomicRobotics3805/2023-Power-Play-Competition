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
    lateinit var leftStartPose: Pose2d
    lateinit var rightStartPose: Pose2d
    // trajectory declarations
    lateinit var leftStartToLowJunction: ParallelTrajectory
    lateinit var leftLowJunctionToTerminal: ParallelTrajectory

    /**
     * Initializes the robot's start positions and trajectories. This is where the trajectories are
     * actually created.
     */
    override fun initialize() {
        super.initialize()
        // start positions
        leftStartPose = Pose2d(30.0, 61.5.switchColor, 270.0.switchColorAngle.toRadians)
        rightStartPose = Pose2d(-30.0, 61.5.switchColor, 270.0.switchColorAngle.toRadians)
        // trajectories
        leftStartToLowJunction = Constants.drive.trajectoryBuilder(leftStartPose)
            .lineTo(Vector2d(23.8, 59.5))
            .build()
        leftLowJunctionToTerminal = Constants.drive.trajectoryBuilder(leftStartToLowJunction.end())
            .lineTo(Vector2d(12.3, 59.5))
            .build()
    }
}