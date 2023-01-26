package org.firstinspires.ftc.teamcode.trajectoryFactory

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.atomicrobotics.cflib.Constants
import com.atomicrobotics.cflib.trajectories.*
import com.atomicrobotics.cflib.Constants.drive as d

@Config
object NewTrajectoryFactory : TrajectoryFactory() {
    // POSES
    // Left side
    var startPoseLeft = Pose2d()
    var stackLocationLeft = Pose2d()
    var stackHighJunctionLocationLeft = Pose2d()
    var preloadHighJunctionLocationLeft = Pose2d()

    var preloadCenterHighJunctionLocationLeft = Pose2d()
    var stackCenterHighJunctionLocationLeft = Pose2d()

    var cyanLocationLeft = Pose2d()
    var magentaLocationLeft = Pose2d()
    var yellowLocationLeft = Pose2d()

    // Right side
    var startPoseRight = Pose2d()
    var stackLocationRight = Pose2d()
    var stackHighJunctionLocationRight = Pose2d()
    var preloadHighJunctionLocationRight = Pose2d()

    var preloadCenterHighJunctionLocationRight = Pose2d()
    var stackCenterHighJunctionLocationRight = Pose2d()

    var cyanLocationRight = Pose2d()
    var magentaLocationRight = Pose2d()
    var yellowLocationRight = Pose2d()

    // TRAJECTORIES
    lateinit var startToHighJunction : ParallelTrajectory
    lateinit var startHighJunctionToStack : ParallelTrajectory
    lateinit var highJunctionToStack : ParallelTrajectory
    lateinit var stackToHighJunction : ParallelTrajectory
    
    lateinit var startToCenterHighJunction : ParallelTrajectory
    lateinit var startCenterHighJunctionToStack : ParallelTrajectory
    lateinit var centerHighJunctionToStack : ParallelTrajectory
    lateinit var stackToCenterHighJunction : ParallelTrajectory

    lateinit var highJunctionToCyanResult : ParallelTrajectory
    lateinit var highJunctionToMagentaResult : ParallelTrajectory
    lateinit var highJunctionToYellowResult : ParallelTrajectory

    lateinit var centerHighJunctionToCyanResult : ParallelTrajectory
    lateinit var centerHighJunctionToMagentaResult : ParallelTrajectory
    lateinit var centerHighJunctionToYellowResult : ParallelTrajectory

    override fun initialize() {
        super.initialize()
        // POSES
        // Left side
        startPoseLeft = Pose2d(35.0, 62.75.switch, 270.0.switchAngle.rad)
        stackLocationLeft = Pose2d(60.75, 12.15.switch, 0.0.switchAngle.rad)
        stackHighJunctionLocationLeft = Pose2d(27.0, 13.25.switch, 0.0.switchAngle.rad)
        preloadHighJunctionLocationLeft = Pose2d(27.0, 11.0.switch, 0.0.switchAngle.rad)

        preloadCenterHighJunctionLocationLeft = Pose2d(0.0, 9.0.switch, 0.0.switchAngle.rad)
        stackCenterHighJunctionLocationLeft = Pose2d(0.0, 10.5.switch, 0.0.switchAngle.rad)

        cyanLocationLeft = Pose2d(35.5.flipAlongX36, 14.0.switch, 270.0.switchAngle.rad)
        magentaLocationLeft = Pose2d(12.0.flipAlongX36, 14.0.switch, 270.0.switchAngle.rad)
        yellowLocationLeft = Pose2d(58.5.flipAlongX36, 14.0.switch, 270.0.switchAngle.rad)

        // Right side
        startPoseRight = Pose2d(35.0, 62.75.switch, 270.0.switchAngle.rad)
        stackLocationRight = Pose2d(59.0, 12.25.switch, 0.0.switchAngle.rad)
        stackHighJunctionLocationRight = Pose2d(26.5, 10.5.switch, 0.0.switchAngle.rad)
        preloadHighJunctionLocationRight = Pose2d(26.0, 9.0.switch, 0.0.switchAngle.rad)

        preloadCenterHighJunctionLocationRight = Pose2d(0.0, 12.0.switch, 0.0.switchAngle.rad)
        stackCenterHighJunctionLocationRight = Pose2d(0.0, 13.25.switch, 0.0.switchAngle.rad)

        cyanLocationRight = Pose2d(33.5, 14.0.switch, 270.0.switchAngle.rad)
        magentaLocationRight = Pose2d(10.0, 14.0.switch, 270.0.switchAngle.rad)
        yellowLocationRight = Pose2d(58.5, 14.0.switch, 270.0.switchAngle.rad)

        // TRAJECTORIES
        startToHighJunction = if(Constants.leftSide) d.trajectoryBuilder(startPoseLeft)
            .splineToSplineHeading(Pose2d(35.5, 30.0.switch, 0.0.switchAngle.rad), 270.0.switchAngle.rad)
            .splineToSplineHeading(Pose2d(35.5, 18.0.switch, 0.0.switchAngle.rad), 270.0.switchAngle.rad)
            .splineToConstantHeading(preloadHighJunctionLocationLeft.v, 180.0.switchAngle.rad)
            .build() else d.trajectoryBuilder(startPoseRight)
            .splineToSplineHeading(Pose2d(34.5, 30.0.switch, 0.0.switchAngle.rad), 270.0.switchAngle.rad)
            .splineToSplineHeading(Pose2d(34.5, 18.0.switch, 0.0.switchAngle.rad), 270.0.switchAngle.rad)
            .splineToConstantHeading(preloadHighJunctionLocationRight.v, 180.0.switchAngle.rad)
            .build()
        startHighJunctionToStack = if(Constants.leftSide) d.trajectoryBuilder(preloadHighJunctionLocationLeft)
            .lineTo(stackLocationLeft.v)
            .build() else d.trajectoryBuilder(preloadHighJunctionLocationRight)
            .lineTo(stackLocationRight.v)
            .build()
        highJunctionToStack = if(Constants.leftSide) d.trajectoryBuilder(stackHighJunctionLocationLeft)
            .lineTo(stackLocationLeft.v)
            .build() else d.trajectoryBuilder(stackHighJunctionLocationRight)
            .lineTo(stackLocationRight.v)
            .build()
        stackToHighJunction = if(Constants.leftSide) d.trajectoryBuilder(stackLocationLeft)
            .lineTo(stackHighJunctionLocationLeft.v)
            .build() else d.trajectoryBuilder(stackLocationRight)
            .lineTo(stackHighJunctionLocationRight.v)
            .build()
        
        startToCenterHighJunction = if(Constants.leftSide) d.trajectoryBuilder(
            startPoseLeft)
            .lineToSplineHeading(Pose2d(35.0, 24.0.switch, 0.0.switchAngle.rad))
            .splineToConstantHeading(preloadCenterHighJunctionLocationLeft.v, 180.0.switchAngle.rad)
            .build() else d.trajectoryBuilder(startPoseRight)
            .lineToSplineHeading(Pose2d(35.0, 24.0.switch, 0.0.switchAngle.rad))
            .splineToConstantHeading(preloadCenterHighJunctionLocationRight.v, 180.0.switchAngle.rad)
            .build()
        startCenterHighJunctionToStack = if(Constants.leftSide) d.trajectoryBuilder(preloadCenterHighJunctionLocationLeft)
            .splineTo(stackLocationLeft.v, 0.0.switchAngle.rad)
            .build() else d.trajectoryBuilder(preloadCenterHighJunctionLocationRight)
            .splineTo(stackLocationRight.v, 0.0.switchAngle.rad)
            .build()
        centerHighJunctionToStack = if(Constants.leftSide) d.trajectoryBuilder(stackCenterHighJunctionLocationLeft)
            .lineToConstantHeading(stackLocationLeft.v)
            .build() else d.trajectoryBuilder(stackCenterHighJunctionLocationRight)
            .lineToConstantHeading(stackLocationRight.v)
            .build()
        stackToCenterHighJunction = if(Constants.leftSide) d.trajectoryBuilder(stackLocationLeft)
            .lineToConstantHeading(stackCenterHighJunctionLocationLeft.v)
            .build() else d.trajectoryBuilder(stackLocationRight)
            .lineToConstantHeading(stackCenterHighJunctionLocationRight.v)
            .build()

        highJunctionToCyanResult = if(Constants.leftSide) d.trajectoryBuilder(stackHighJunctionLocationLeft)
            .lineToLinearHeading(cyanLocationLeft)
            .build() else d.trajectoryBuilder(stackHighJunctionLocationRight)
            .lineToLinearHeading(cyanLocationRight)
            .build()
        highJunctionToMagentaResult = if(Constants.leftSide) d.trajectoryBuilder(stackHighJunctionLocationLeft)
            .lineToLinearHeading(magentaLocationLeft)
            .build() else d.trajectoryBuilder(stackHighJunctionLocationRight)
            .lineToLinearHeading(magentaLocationRight)
            .build()
        highJunctionToYellowResult = if(Constants.leftSide) d.trajectoryBuilder(stackHighJunctionLocationLeft)
            .lineToLinearHeading(yellowLocationLeft)
            .build() else d.trajectoryBuilder(stackHighJunctionLocationRight)
            .lineToLinearHeading(yellowLocationRight)
            .build()

        centerHighJunctionToCyanResult = if(Constants.leftSide) d.trajectoryBuilder(stackCenterHighJunctionLocationLeft)
            .splineToLinearHeading(cyanLocationLeft, 20.0.switchAngle.rad)
            .build() else d.trajectoryBuilder(stackCenterHighJunctionLocationRight)
            .splineToLinearHeading(cyanLocationRight, 20.0.switchAngle.rad)
            .build()
        centerHighJunctionToMagentaResult = if(Constants.leftSide) d.trajectoryBuilder(stackCenterHighJunctionLocationLeft)
            .lineToLinearHeading(magentaLocationLeft)
            .build() else d.trajectoryBuilder(stackCenterHighJunctionLocationRight)
            .lineToLinearHeading(magentaLocationRight)
            .build()
        centerHighJunctionToYellowResult = if(Constants.leftSide) d.trajectoryBuilder(stackCenterHighJunctionLocationLeft)
            .lineToLinearHeading(yellowLocationLeft)
            .build() else d.trajectoryBuilder(stackCenterHighJunctionLocationRight)
            .lineToLinearHeading(yellowLocationRight)
            .build()
    }
}