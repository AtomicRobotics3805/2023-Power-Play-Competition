package org.firstinspires.ftc.teamcode.tuning.drivetrain

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.atomicrobotics.cflib.CommandScheduler
import com.atomicrobotics.cflib.Constants
import com.atomicrobotics.cflib.TelemetryController
import com.atomicrobotics.cflib.driving.Turn
import com.atomicrobotics.cflib.driving.drivers.MecanumDrive
import com.atomicrobotics.cflib.driving.localizers.TwoWheelOdometryLocalizer
import com.atomicrobotics.cflib.trajectories.toRadians
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.tuning.constants.TuningMecanumDriveConstants
import org.firstinspires.ftc.teamcode.tuning.constants.TuningOdometryConstants

/*
 * This is a simple routine to test turning capabilities.
 */
@Config
@Autonomous(group = "drive")
class TurnTest : LinearOpMode() {
    @Throws(InterruptedException::class)
    override fun runOpMode() {
        Constants.opMode = this
        Constants.drive = MecanumDrive(
            TuningMecanumDriveConstants,
            TwoWheelOdometryLocalizer(TuningOdometryConstants()),
        ) { Pose2d() }
        CommandScheduler.registerSubsystems(Constants.drive, TelemetryController)
        waitForStart()
        CommandScheduler.scheduleCommand(Constants.drive.turn(ANGLE.toRadians, Turn.TurnType.RELATIVE))
        while (opModeIsActive()) {
            CommandScheduler.run()
        }
    }

    companion object {
        var ANGLE = 90.0 // deg
    }
}