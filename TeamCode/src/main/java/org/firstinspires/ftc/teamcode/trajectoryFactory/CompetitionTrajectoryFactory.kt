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
object CompetitionTrajectoryFactory : TrajectoryFactory() {

    // start position declarations
    lateinit var legalStartPose: Pose2d



    // 26 Point
    // trajectory declarations
    lateinit var startToLowJunction: ParallelTrajectory
    lateinit var lowJunctionToSignalLeft: ParallelTrajectory
    lateinit var lowJunctionToSignalRight: ParallelTrajectory

    // Signal results
    lateinit var signalResultRed: ParallelTrajectory // Left
    lateinit var signalResultGreen: ParallelTrajectory // Center
    lateinit var signalResultBlue: ParallelTrajectory // Right


    // 30 Point
    lateinit var startToSignalLeft: ParallelTrajectory
    lateinit var startToSignalRight: ParallelTrajectory

    lateinit var signalToHighJunction: ParallelTrajectory

    // STACK SCORING
    lateinit var stack: Pose2d
    lateinit var highJunction: Vector2d

    lateinit var stackToHighJunction: ParallelTrajectory
    lateinit var highJunctionToStack: ParallelTrajectory

    lateinit var centeredStartPose: Pose2d
    lateinit var centerStartToHighJunction: ParallelTrajectory

    lateinit var highJunctionToYellowResult: ParallelTrajectory
    lateinit var highJunctionToCyanResult: ParallelTrajectory
    lateinit var highJunctionToMagentaResult: ParallelTrajectory

    /**
     * Initializes the robot's start positions and trajectories. This is where the trajectories are
     * actually created.
     */
    override fun initialize() {
        super.initialize()
        // 26 Point
        // start positions
        legalStartPose = Pose2d(33.0, 62.75.switchColor, 270.0.switchColorAngle.toRadians)

        // trajectories
        startToLowJunction = Constants.drive.trajectoryBuilder(legalStartPose)
            .lineTo(Vector2d(23.0, 53.5.switchColor))
            .build()
        lowJunctionToSignalLeft = Constants.drive.trajectoryBuilder(startToLowJunction.end())
            .lineTo(Vector2d(23.9, 56.4.switchColor))
            .splineToConstantHeading(Vector2d(32.0, 60.0.switchColor), 300.0.switchColorAngle.toRadians)
            .splineToSplineHeading(Pose2d(34.0, 47.0.switchColor, 235.0.switchColorAngle.toRadians), 250.0.switchColorAngle.toRadians)
            .build()

        lowJunctionToSignalRight = Constants.drive.trajectoryBuilder(startToLowJunction.end())
            .lineTo(Vector2d(23.9, 56.4.switchColor))
            .splineToConstantHeading(Vector2d(32.0, 60.0.switchColor), 300.0.switchColorAngle.toRadians)
            .splineToSplineHeading(Pose2d(34.0, 47.0.switchColor, 305.0.switchColorAngle.toRadians), 250.0.switchColorAngle.toRadians)
            .build()

        signalResultBlue = Constants.drive.trajectoryBuilder(if(Constants.color == Constants.Color.RED) lowJunctionToSignalRight.end() else lowJunctionToSignalLeft.end())
            .splineToSplineHeading(Pose2d(12.3.flipAlongX36, 37.0.switchColor, 270.0.switchColorAngle.toRadians), 180.0.switchApproachTangentAngle.toRadians)
            .build()
        signalResultRed = Constants.drive.trajectoryBuilder(if(Constants.color == Constants.Color.RED) lowJunctionToSignalRight.end() else lowJunctionToSignalLeft.end())
            .splineToSplineHeading(Pose2d(59.5.flipAlongX36, 37.0.switchColor, 270.0.switchColorAngle.toRadians), 0.0.switchApproachTangentAngle.toRadians)
            .build()
        signalResultGreen = Constants.drive.trajectoryBuilder(if(Constants.color == Constants.Color.RED) lowJunctionToSignalRight.end() else lowJunctionToSignalLeft.end())
            .lineToSplineHeading(Pose2d(35.0, 37.0.switchColor, 270.0.switchColorAngle.toRadians))
            .build()

        // 30 Point
        startToSignalLeft = Constants.drive.trajectoryBuilder(legalStartPose)
            .splineToSplineHeading(Pose2d(34.0, 48.5.switchColor, 250.0.switchColorAngle.toRadians), 250.0.switchColorAngle.toRadians)
            .build()
        startToSignalRight = Constants.drive.trajectoryBuilder(legalStartPose)
            .splineToSplineHeading(Pose2d(34.0, 48.5.switchColor, 300.0.switchColorAngle.toRadians), 250.0.switchColorAngle.toRadians)
            .build()

// 27.5, 0.2
        signalToHighJunction = Constants.drive.trajectoryBuilder(if(Constants.color == Constants.Color.RED) startToSignalRight.end() else startToSignalLeft.end())
            .lineToSplineHeading(Pose2d(34.0, 5.6.switchColor, 180.0.switchColorAngle.toRadians))
            .splineToConstantHeading(Vector2d(29.0, 2.0), 180.0.toRadians)
            .build()

        // 60.0, 11.8
// 24.0, 18.0
        stack = Pose2d(59.0, 12.0.switchColor, 0.0.switchColorAngle.toRadians)
        highJunction = Vector2d(27.75, 9.9.switchColor)

        centeredStartPose = Pose2d(35.5, 62.75.switchColor, 270.0.switchColorAngle.toRadians)

        stackToHighJunction = Constants.drive.trajectoryBuilder(stack)
            .lineTo(Vector2d(26.75, 13.0.switchColor))
            .build()
        highJunctionToStack = Constants.drive.trajectoryBuilder(stackToHighJunction.end())
            .lineTo(Vector2d(62.0, 12.0.switchColor))
            .build()




        centerStartToHighJunction = Constants.drive.trajectoryBuilder(centeredStartPose)
            .splineToSplineHeading(Pose2d(35.5, 18.0.switchColor, 0.0.switchColorAngle.toRadians), 270.0.switchColorAngle.toRadians)
            .splineToConstantHeading(Vector2d(26.75, 13.0.switchColor), 180.0.switchColorAngle.toRadians)
            .build()

        highJunctionToMagentaResult = Constants.drive.trajectoryBuilder(centerStartToHighJunction.end())
            .lineToLinearHeading(Pose2d(12.0, 12.0.switchColor, 270.0.switchColorAngle.toRadians))
            .build()
        highJunctionToCyanResult = Constants.drive.trajectoryBuilder(centerStartToHighJunction.end())
            .lineToLinearHeading(Pose2d(35.5, 12.0.switchColor, 270.0.switchColorAngle.toRadians))
            .build()
        highJunctionToYellowResult = Constants.drive.trajectoryBuilder(centerStartToHighJunction.end())
            .lineToLinearHeading(Pose2d(58.5, 12.0.switchColor, 270.0.switchColorAngle.toRadians))
            .build()
    }
}