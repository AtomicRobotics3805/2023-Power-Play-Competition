package org.firstinspires.ftc.teamcode.opmodes

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.atomicrobotics.unused.Constants
import com.atomicrobotics.unused.driving.drivers.MecanumDrive
import com.atomicrobotics.unused.driving.localizers.TwoWheelOdometryLocalizer
import com.atomicrobotics.unused.opmodes.TeleOpMode
import org.firstinspires.ftc.teamcode.controls.ExampleControls
import org.firstinspires.ftc.teamcode.drive.CompetitionMecanumDriveConstants
import com.atomicrobotics.unused.trajectories.localizers.CompetitionOdometryConstants
import org.firstinspires.ftc.teamcode.routines.Routines
import org.firstinspires.ftc.teamcode.trajectoryFactory.CompetitionTrajectoryFactory

/**
 * This class is an example of how you can create an TeleOp OpMode. Everything is handled by the
 * TeleOpMode parent class, so all you have to do is pass in the constructor parameters.
 */

@TeleOp(name = "Testing TeleOp OpMode", group="Testing")
class ExampleTeleOpMode : TeleOpMode(
    ExampleControls,
    Constants.Color.UNKNOWN,
    CompetitionTrajectoryFactory,
    { Routines.teleOpStartRoutine },
    null,
    MecanumDrive(
        CompetitionMecanumDriveConstants,
        TwoWheelOdometryLocalizer(CompetitionOdometryConstants()),
    ) { Constants.endPose ?: Pose2d() },
)