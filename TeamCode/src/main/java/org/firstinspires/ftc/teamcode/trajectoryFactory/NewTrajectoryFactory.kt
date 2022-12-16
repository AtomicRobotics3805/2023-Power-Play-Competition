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
    //@JvmField
    var startPoseLeft = Pose2d()
    //@JvmField
    var stackLocationLeft = Pose2d()
    //@JvmField
    var stackHighJunctionLocationLeft = Pose2d()
    //@JvmField
    var preloadHighJunctionLocationLeft = Pose2d()

    //@JvmField
    var cyanLocationLeft = Pose2d()
    //@JvmField
    var magentaLocationLeft = Pose2d()
    //@JvmField
    var yellowLocationLeft = Pose2d()

    // Right side
    //@JvmField
    var startPoseRight = Pose2d()
    //@JvmField
    var stackLocationRight = Pose2d()
    //@JvmField
    var stackHighJunctionLocationRight = Pose2d()
    //@JvmField
    var preloadHighJunctionLocationRight = Pose2d()

    //@JvmField
    var cyanLocationRight = Pose2d()
    //@JvmField
    var magentaLocationRight = Pose2d()
    //@JvmField
    var yellowLocationRight = Pose2d()

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
        // Left side
        startPoseLeft = Pose2d(35.0, 62.75.switchColor, 270.0.switchColorAngle.toRadians)
        stackLocationLeft = Pose2d(61.5, 12.25.switchColor, 0.0.switchColorAngle.toRadians)
        stackHighJunctionLocationLeft = Pose2d(27.0, 12.5.switchColor, 0.0.switchColorAngle.toRadians)
        preloadHighJunctionLocationLeft = Pose2d(27.0, 11.0.switchColor, 0.0.switchColorAngle.toRadians)

        cyanLocationLeft = Pose2d(35.5.flipAlongX36, 14.0.switchColor, 270.0.switchColorAngle.toRadians)
        magentaLocationLeft = Pose2d(12.0.flipAlongX36, 14.0.switchColor, 270.0.switchColorAngle.toRadians)
        yellowLocationLeft = Pose2d(58.5.flipAlongX36, 14.0.switchColor, 270.0.switchColorAngle.toRadians)

        // Right side
        startPoseRight = Pose2d(35.0, 62.75.switchColor, 270.0.switchColorAngle.toRadians)
        stackLocationRight = Pose2d(59.5, 12.25.switchColor, 0.0.switchColorAngle.toRadians)
        stackHighJunctionLocationRight = Pose2d(26.5, 10.5.switchColor, 0.0.switchColorAngle.toRadians)
        preloadHighJunctionLocationRight = Pose2d(27.0, 9.0.switchColor, 0.0.switchColorAngle.toRadians)

        cyanLocationRight = Pose2d(35.5.flipAlongX36, 14.0.switchColor, 270.0.switchColorAngle.toRadians)
        magentaLocationRight = Pose2d(12.0.flipAlongX36, 14.0.switchColor, 270.0.switchColorAngle.toRadians)
        yellowLocationRight = Pose2d(58.5.flipAlongX36, 14.0.switchColor, 270.0.switchColorAngle.toRadians)

        // TRAJECTORIES
        startToHighJunction = if(Constants.color == Constants.Color.BLUE) d.trajectoryBuilder(startPoseLeft)
            .splineToSplineHeading(Pose2d(35.5, 30.0.switchColor, 0.0.switchColorAngle.toRadians), 270.0.switchColorAngle.toRadians)
            .splineToSplineHeading(Pose2d(35.5, 18.0.switchColor, 0.0.switchColorAngle.toRadians), 270.0.switchColorAngle.toRadians)
            .splineToConstantHeading(preloadHighJunctionLocationLeft.vec(), 180.0.switchColorAngle.toRadians)
            .build() else d.trajectoryBuilder(startPoseRight)
            .splineToSplineHeading(Pose2d(35.5, 30.0.switchColor, 0.0.switchColorAngle.toRadians), 270.0.switchColorAngle.toRadians)
            .splineToSplineHeading(Pose2d(35.5, 18.0.switchColor, 0.0.switchColorAngle.toRadians), 270.0.switchColorAngle.toRadians)
            .splineToConstantHeading(preloadHighJunctionLocationRight.vec(), 180.0.switchColorAngle.toRadians)
            .build()
        startHighJunctionToStack = if(Constants.color == Constants.Color.BLUE) d.trajectoryBuilder(preloadHighJunctionLocationLeft)
            .lineTo(stackLocationLeft.vec())
            .build() else d.trajectoryBuilder(preloadHighJunctionLocationRight)
            .lineTo(stackLocationRight.vec())
            .build()
        highJunctionToStack = if(Constants.color == Constants.Color.BLUE) d.trajectoryBuilder(stackHighJunctionLocationLeft)
            .lineTo(stackLocationLeft.vec())
            .build() else d.trajectoryBuilder(stackHighJunctionLocationRight)
            .lineTo(stackLocationRight.vec())
            .build()
        stackToHighJunction = if(Constants.color == Constants.Color.BLUE) d.trajectoryBuilder(stackLocationLeft)
            .lineTo(stackHighJunctionLocationLeft.vec())
            .build() else d.trajectoryBuilder(stackLocationRight)
            .lineTo(stackHighJunctionLocationRight.vec())
            .build()
        highJunctionToCyanResult = if(Constants.color == Constants.Color.BLUE) d.trajectoryBuilder(stackHighJunctionLocationLeft)
            .lineToLinearHeading(cyanLocationLeft)
            .build() else d.trajectoryBuilder(stackHighJunctionLocationRight)
            .lineToLinearHeading(cyanLocationRight)
            .build()
        highJunctionToMagentaResult = if(Constants.color == Constants.Color.BLUE) d.trajectoryBuilder(stackHighJunctionLocationLeft)
            .lineToLinearHeading(magentaLocationLeft)
            .build() else d.trajectoryBuilder(stackHighJunctionLocationRight)
            .lineToLinearHeading(magentaLocationRight)
            .build()
        highJunctionToYellowResult = if(Constants.color == Constants.Color.BLUE) d.trajectoryBuilder(stackHighJunctionLocationLeft)
            .lineToLinearHeading(yellowLocationLeft)
            .build() else d.trajectoryBuilder(stackHighJunctionLocationRight)
            .lineToLinearHeading(yellowLocationRight)
            .build()
    }
}