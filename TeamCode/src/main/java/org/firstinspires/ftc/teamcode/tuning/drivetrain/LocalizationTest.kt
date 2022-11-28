package org.firstinspires.ftc.teamcode.tuning.drivetrain

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.atomicrobotics.cflib.CommandScheduler
import com.atomicrobotics.cflib.Constants.drive
import com.atomicrobotics.cflib.Constants.opMode
import com.atomicrobotics.cflib.TelemetryController
import com.atomicrobotics.cflib.driving.drivers.MecanumDrive
import com.atomicrobotics.cflib.driving.localizers.TwoWheelOdometryLocalizer
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.tuning.constants.TuningMecanumDriveConstants
import org.firstinspires.ftc.teamcode.tuning.constants.TuningOdometryConstants

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@Config
@TeleOp(group = "drive")
class LocalizationTest : LinearOpMode() {
    @Throws(InterruptedException::class)
    override fun runOpMode() {
        opMode = this
        drive = MecanumDrive(
            TuningMecanumDriveConstants,
            TwoWheelOdometryLocalizer(TuningOdometryConstants()),
        ) { Pose2d() }
        CommandScheduler.registerSubsystems(TelemetryController, drive)
        waitForStart()
        CommandScheduler.scheduleCommand(drive.driverControlled(gamepad1))
        while (!isStopRequested) {
            CommandScheduler.run()
        }
    }
}