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

    lateinit var leftSensorStartPose: Pose2d
    // trajectory declarations
    lateinit var leftStartToLowJunction: ParallelTrajectory
    lateinit var leftLowJunctionToTerminal: ParallelTrajectory

    lateinit var leftSensorStartToLeftSignal: ParallelTrajectory

    lateinit var leftSignalToLeftParkZone: ParallelTrajectory

    // complex path
    lateinit var leftComplexStartPose: Pose2d
    lateinit var leftComplexStartToLowJunction: ParallelTrajectory
    lateinit var leftComplexLowJunctionToSignal: ParallelTrajectory

    // Signal results
    lateinit var leftSignalToConeStack: ParallelTrajectory
    lateinit var leftSignalResultRed: ParallelTrajectory // Left
    lateinit var leftSignalResultGreen: ParallelTrajectory // Center
    lateinit var leftSignalResultBlue: ParallelTrajectory // Right
    lateinit var coneStackToHighJunction: ParallelTrajectory
    lateinit var highJunctionToConeStack: ParallelTrajectory

    lateinit var forward: ParallelTrajectory
    lateinit var strafeRight: ParallelTrajectory

    /**
     * Initializes the robot's start positions and trajectories. This is where the trajectories are
     * actually created.
     */
    override fun initialize() {
        super.initialize()
        // start positions
        leftStartPose = Pose2d(30.0, 61.5.switchColor, 270.0.switchColorAngle.toRadians)
        rightStartPose = Pose2d(-30.0, 61.5.switchColor, 270.0.switchColorAngle.toRadians)

        leftSensorStartPose = Pose2d(34.5, 61.5.switchColor, 270.0.switchColorAngle.toRadians)


        // trajectories
        leftStartToLowJunction = Constants.drive.trajectoryBuilder(leftStartPose)
            .lineTo(Vector2d(23.8, 50.5.switchColor))
            .build()
        leftLowJunctionToTerminal = Constants.drive.trajectoryBuilder(leftStartToLowJunction.end())
            .lineTo(Vector2d(12.3, 59.5.switchColor))
            .build()
        leftSensorStartToLeftSignal = Constants.drive.trajectoryBuilder(leftSensorStartPose)
            .lineTo(Vector2d(34.5, 46.8))
            .build()

        forward = Constants.drive.trajectoryBuilder(Pose2d())
            .forward(10.0)
            .build()
        strafeRight = Constants.drive.trajectoryBuilder(forward.end())
            .strafeRight(10.0)
            .build()



        // 36.4
        leftSignalToLeftParkZone = Constants.drive.trajectoryBuilder(leftSensorStartToLeftSignal.end())
            .splineTo(Vector2d(58.3, 35.7), 0.0.toRadians)
            .build()
        // 58.3, 35.7


        // COMPLEX PATH
        leftComplexStartPose = Pose2d(23.5, 61.5.switchColor, 0.0.switchColorAngle.toRadians)
        leftComplexStartToLowJunction = Constants.drive.trajectoryBuilder(leftComplexStartPose)
            .lineTo(Vector2d(23.5, 55.5))
            .build()
        leftComplexLowJunctionToSignal = Constants.drive.trajectoryBuilder(leftComplexStartToLowJunction.end())
            .splineToLinearHeading(Pose2d(34.5, 46.8, 270.0.toRadians), 270.0.toRadians)
            .build()
        leftSignalToConeStack = Constants.drive.trajectoryBuilder(leftComplexLowJunctionToSignal.end())
            .lineTo(Vector2d(34.5, 25.5))
            .splineTo(Vector2d(60.0, 12.0), 0.0.toRadians)
            .build()
        coneStackToHighJunction = Constants.drive.trajectoryBuilder(leftSignalToConeStack.end())
            .lineTo(Vector2d(23.5, 10.0))
            .build()
        highJunctionToConeStack = Constants.drive.trajectoryBuilder(coneStackToHighJunction.end())
            .lineTo(Vector2d(60.0, 12.0))
            .build()

        // ~ 12.0
     //   leftSignalResultRed = Constants.drive.trajectoryBuilder(leftComplexLowJunctionToSignal.end())
            //.splineToSplineHeading(Pose2d(34.5, 23.0, 0.0.switchColorAngle.toRadians), 270.0.switchColorAngle.toRadians)
         //   .lineTo(Vector2d(34.5, 25.5))
          //  .splineTo(Vector2d(60.0, 12.0), 0.0.toRadians)
         //   .build()

        leftSignalResultBlue = Constants.drive.trajectoryBuilder(highJunctionToConeStack.end())
            .lineTo(Vector2d(11.5, 11.5))
            .build()

        leftSignalResultRed = Constants.drive.trajectoryBuilder(highJunctionToConeStack.end())
            .lineTo(Vector2d(58.0, 30.5))
            .build()


        // Start @ 23.5, 61.5
        // Drive forward to 23.5, 59.5 (low junction)
        // Score on low junction
        // Drive to 34.5, 46.8 (signal)
        // Detect signal position
        // Drive to correct location

    }
}