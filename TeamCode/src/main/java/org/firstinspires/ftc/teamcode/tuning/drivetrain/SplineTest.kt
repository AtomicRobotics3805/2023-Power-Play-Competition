package org.firstinspires.ftc.teamcode.tuning.drivetrain

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.atomicrobotics.cflib.CommandScheduler
import com.atomicrobotics.cflib.Constants
import com.atomicrobotics.cflib.Constants.drive
import com.atomicrobotics.cflib.TelemetryController
import com.atomicrobotics.cflib.driving.drivers.MecanumDrive
import com.atomicrobotics.cflib.driving.localizers.TwoWheelOdometryLocalizer
import com.atomicrobotics.cflib.sequential
import com.atomicrobotics.cflib.trajectories.ParallelTrajectory
import com.atomicrobotics.cflib.trajectories.toRadians
import com.atomicrobotics.cflib.utilCommands.Delay
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.tuning.constants.TuningMecanumDriveConstants
import org.firstinspires.ftc.teamcode.tuning.constants.TuningOdometryConstants

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
class SplineTest : LinearOpMode() {
    @Throws(InterruptedException::class)
    override fun runOpMode() {
        Constants.opMode = this
        drive = MecanumDrive(
            TuningMecanumDriveConstants,
            TwoWheelOdometryLocalizer(TuningOdometryConstants()),
        ) { Pose2d() }
        CommandScheduler.registerSubsystems(drive, TelemetryController)
        val forwardTrajectory: ParallelTrajectory = drive.trajectoryBuilder(Pose2d())
            .splineTo(Vector2d(30.0, 30.0), 0.0)
            .build()
        val reverseTrajectory: ParallelTrajectory = drive.trajectoryBuilder(forwardTrajectory.end(), true)
            .splineTo(Vector2d(0.0, 0.0), 180.0.toRadians)
            .build()
        waitForStart()
        CommandScheduler.scheduleCommand(sequential {
            +drive.followTrajectory(forwardTrajectory)
            +Delay(2.0)
            +drive.followTrajectory(reverseTrajectory)
        })
        while (opModeIsActive()) {
            CommandScheduler.run()
        }
    }
}