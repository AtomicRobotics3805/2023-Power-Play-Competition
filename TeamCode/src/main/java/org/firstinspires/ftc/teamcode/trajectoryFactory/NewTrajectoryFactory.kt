package org.firstinspires.ftc.teamcode.trajectoryFactory

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.atomicrobotics.cflib.Constants
import com.atomicrobotics.cflib.Constants.drive as d
import com.atomicrobotics.cflib.trajectories.*

object NewTrajectoryFactory : TrajectoryFactory() {

    fun tb(startPose: Pose2d, reversed: Boolean = false) = d.trajectoryBuilder(startPose, reversed)
    fun tb(startPose: Pose2d, startHeading: Double) = d.trajectoryBuilder(startPose, startHeading)

    // POSES
    lateinit var startPose : Pose2d
    lateinit var stackLocation: Pose2d
    lateinit var stackHighJunctionLocation : Pose2d
    lateinit var preloadHighJunctionLocation : Pose2d

    lateinit var cyanLocation : Pose2d
    lateinit var magentaLocation : Pose2d
    lateinit var yellowLocation : Pose2d

    // TRAJECTORIES
    lateinit var startToHighJunction : ParallelTrajectory
    lateinit var startHighJunctionToStack : ParallelTrajectory
    lateinit var highJunctionToStack : ParallelTrajectory
    lateinit var stackToHighJunction : ParallelTrajectory

    lateinit var highJunctionToCyanResult : ParallelTrajectory
    lateinit var highJunctionToMagentaResult : ParallelTrajectory
    lateinit var highJunctionToYellowResult : ParallelTrajectory

    override fun initialize() {
        super.initialize()

        // POSES
        startPose = Pose2d(35.0, 62.75.switchColor, 270.0.switchColorAngle.toRadians)
        stackLocation = Pose2d(61.0, 13.5.switchColor, 0.0.switchColorAngle.toRadians)
        stackHighJunctionLocation = Pose2d(27.0, 14.0.switchColor, 0.0.switchColorAngle.toRadians)
        preloadHighJunctionLocation = Pose2d(61.0, 13.5.switchColor, 0.0.switchColorAngle.toRadians)

        cyanLocation = Pose2d(35.5.flipAlongX36, 14.0.switchColor, 270.0.switchColorAngle.toRadians)
        magentaLocation = Pose2d(12.0.flipAlongX36, 14.0.switchColor, 270.0.switchColorAngle.toRadians)
        yellowLocation = Pose2d(58.5.flipAlongX36, 14.0.switchColor, 270.0.switchColorAngle.toRadians)

        // TRAJECTORIES
        startToHighJunction = tb(startPose)
            .splineToSplineHeading(Pose2d(35.5, 30.0.switchColor, 0.0.switchColorAngle.toRadians), 270.0.switchColorAngle.toRadians)
            .splineToSplineHeading(Pose2d(35.5, 18.0.switchColor, 0.0.switchColorAngle.toRadians), 270.0.switchColorAngle.toRadians)
            .splineToConstantHeading(preloadHighJunctionLocation.vec(), 180.0.switchColorAngle.toRadians)
            .build()

        startHighJunctionToStack = tb(preloadHighJunctionLocation)
            .lineTo(stackLocation.vec())
            .build()
        highJunctionToStack = tb(stackHighJunctionLocation)
            .lineTo(stackLocation.vec())
            .build()
        stackToHighJunction = tb(stackLocation)
            .lineTo(stackHighJunctionLocation.vec())
            .build()

        highJunctionToCyanResult = tb(stackHighJunctionLocation)
            .lineToLinearHeading(cyanLocation)
            .build()
        highJunctionToMagentaResult = tb(stackHighJunctionLocation)
            .lineToLinearHeading(magentaLocation)
            .build()
        highJunctionToYellowResult = tb(stackHighJunctionLocation)
            .lineToLinearHeading(yellowLocation)
            .build()
    }
}