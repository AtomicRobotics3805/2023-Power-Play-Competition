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

    // Start Positions
    lateinit var legalStartPose: Pose2d
    lateinit var centeredStartPose: Pose2d

    // Low Junction -> Signal -> Park (26 Point: 23 auto points, 3 endgame points)
    lateinit var startToLowJunction: ParallelTrajectory
    lateinit var lowJunctionToSignalLeft: ParallelTrajectory
    lateinit var lowJunctionToSignalRight: ParallelTrajectory

    lateinit var signalResultRed: ParallelTrajectory // Left
    lateinit var signalResultGreen: ParallelTrajectory // Center
    lateinit var signalResultBlue: ParallelTrajectory // Right

    // High Junction -> Stack -> High Junction -> Stack -> High Junction -> Park (50 Point: 35 auto points, 15 endgame points)
    lateinit var startHighJunctionToStackLeft: ParallelTrajectory
    lateinit var highJunctionToStackLeft: ParallelTrajectory
    lateinit var stackToHighJunctionLeft: ParallelTrajectory

    lateinit var startHighJunctionToStackRight: ParallelTrajectory
    lateinit var highJunctionToStackRight: ParallelTrajectory
    lateinit var stackToHighJunctionRight: ParallelTrajectory
    lateinit var centerStartToHighJunctionRight: ParallelTrajectory

    lateinit var centerStartToHighJunctionLeft: ParallelTrajectory

    lateinit var centerStartToLowJunction: ParallelTrajectory

    lateinit var highJunctionToYellowResult: ParallelTrajectory
    lateinit var highJunctionToCyanResult: ParallelTrajectory
    lateinit var highJunctionToMagentaResult: ParallelTrajectory

    lateinit var lowJunctionToYellowResult: ParallelTrajectory
    lateinit var lowJunctionToCyanResult: ParallelTrajectory
    lateinit var lowJunctionToMagentaResult: ParallelTrajectory

    /**
     * Initializes the robot's start positions and trajectories. This is where the trajectories are
     * actually created.
     */
    override fun initialize() {
        super.initialize()
        // 26 Point
        // start positions
        legalStartPose = Pose2d(33.0, 62.75.switchColor, 270.0.switchColorAngle.toRadians)

        centeredStartPose = Pose2d(35.0, 62.75.switchColor, 270.0.switchColorAngle.toRadians)

        // trajectories
        startToLowJunction = Constants.drive.trajectoryBuilder(legalStartPose)
            .lineTo(Vector2d(23.0, 53.5.switchColor))
            .build()

        centerStartToLowJunction = Constants.drive.trajectoryBuilder(centeredStartPose)
            .lineTo(Vector2d(23.0, 53.5.switchColor))
            .build()

        lowJunctionToYellowResult = Constants.drive.trajectoryBuilder(centerStartToLowJunction.end())
            .lineTo(Vector2d(23.9, 56.4.switchColor))
            .splineToConstantHeading(Vector2d(35.0, 56.5.switchColor), 270.0.switchColorAngle.toRadians)
            .splineToConstantHeading(Vector2d(35.0, 45.0.switchColor), 270.0.switchColorAngle.toRadians)
            .splineToLinearHeading(Pose2d(59.0.flipAlongX36, 34.0.switchColor, 270.0.switchColorAngle.toRadians), 0.0.switchApproachTangentAngle.toRadians)
            .build()

        lowJunctionToCyanResult = Constants.drive.trajectoryBuilder(centerStartToLowJunction.end())
            .lineTo(Vector2d(23.9, 56.4.switchColor))
            .splineToConstantHeading(Vector2d(35.0, 56.5.switchColor), 270.0.switchColorAngle.toRadians)
            .splineToConstantHeading(Vector2d(35.0, 34.0.switchColor), 270.0.switchColorAngle.toRadians)
            .build()

        lowJunctionToMagentaResult = Constants.drive.trajectoryBuilder(centerStartToLowJunction.end())
            .lineTo(Vector2d(23.9, 56.4.switchColor))
            .splineToConstantHeading(Vector2d(35.0, 56.5.switchColor), 270.0.switchColorAngle.toRadians)
            .splineToConstantHeading(Vector2d(35.0, 45.0.switchColor), 270.0.switchColorAngle.toRadians)
            .splineToLinearHeading(Pose2d(12.3.flipAlongX36, 34.0.switchColor, 270.0.switchColorAngle.toRadians), 180.0.switchApproachTangentAngle.toRadians)
            .build()

        // yellow is left
        // cyan is center
        // magenta is right

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



        centerStartToHighJunctionLeft = Constants.drive.trajectoryBuilder(centeredStartPose)
                .splineToSplineHeading(Pose2d(35.5, 30.0.switchColor, 0.0.switchColorAngle.toRadians), 270.0.switchColorAngle.toRadians)
                .splineToSplineHeading(Pose2d(35.5, 18.0.switchColor, 0.0.switchColorAngle.toRadians), 270.0.switchColorAngle.toRadians)
                .splineToConstantHeading(Vector2d(27.0, 12.5.switchColor), 180.0.switchColorAngle.toRadians)
                .build()
        startHighJunctionToStackLeft = Constants.drive.trajectoryBuilder(centerStartToHighJunctionLeft.end())
            .lineTo(Vector2d(61.0, 13.5.switchColor))
            .build()
        stackToHighJunctionLeft = Constants.drive.trajectoryBuilder(startHighJunctionToStackLeft.end())
                .lineTo(Vector2d(27.0, 14.0.switchColor))
                .build()
        highJunctionToStackLeft = Constants.drive.trajectoryBuilder(stackToHighJunctionLeft.end())
                .lineTo(startHighJunctionToStackLeft.end().vec())
                .build()


        centerStartToHighJunctionRight = Constants.drive.trajectoryBuilder(centeredStartPose)
            .splineToSplineHeading(Pose2d(35.5, 30.0.switchColor, 0.0.switchColorAngle.toRadians), 270.0.switchColorAngle.toRadians)
            .splineToSplineHeading(Pose2d(35.5, 18.0.switchColor, 0.0.switchColorAngle.toRadians), 270.0.switchColorAngle.toRadians)
            .splineToConstantHeading(Vector2d(29.5, 7.5.switchColor), 180.0.switchColorAngle.toRadians)
            .build()
        stackToHighJunctionRight = Constants.drive.trajectoryBuilder(Pose2d(63.0, 10.5.switchColor, 0.0.switchColorAngle.toRadians))
            .lineTo(Vector2d(29.5, 9.0.switchColor))
            .build()
        highJunctionToStackRight = Constants.drive.trajectoryBuilder(stackToHighJunctionRight.end())
            .lineTo(Vector2d(63.0, 10.5.switchColor))
            .build()
        startHighJunctionToStackRight = Constants.drive.trajectoryBuilder(centerStartToHighJunctionRight.end())
             .lineTo(Vector2d(63.0, 10.5.switchColor))
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

        highJunctionToMagentaResult = Constants.drive.trajectoryBuilder(if(Constants.color == Constants.Color.BLUE) stackToHighJunctionLeft.end() else stackToHighJunctionRight.end())
            .lineToLinearHeading(Pose2d(12.0.flipAlongX36, 14.0.switchColor, 270.0.switchColorAngle.toRadians))
            .build()
        highJunctionToCyanResult = Constants.drive.trajectoryBuilder(if(Constants.color == Constants.Color.BLUE) stackToHighJunctionLeft.end() else stackToHighJunctionRight.end())
            .lineToLinearHeading(Pose2d(35.5.flipAlongX36, 14.0.switchColor, 270.0.switchColorAngle.toRadians))
            .build()
        highJunctionToYellowResult = Constants.drive.trajectoryBuilder(if(Constants.color == Constants.Color.BLUE) stackToHighJunctionLeft.end() else stackToHighJunctionRight.end())
            .lineToLinearHeading(Pose2d(58.5.flipAlongX36, 14.0.switchColor, 270.0.switchColorAngle.toRadians))
            .build()



    }
}