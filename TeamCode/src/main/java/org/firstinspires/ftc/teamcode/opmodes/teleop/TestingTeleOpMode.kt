package org.firstinspires.ftc.teamcode.opmodes

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.atomicrobotics.cflib.Constants
import com.atomicrobotics.cflib.driving.drivers.MecanumDrive
import com.atomicrobotics.cflib.driving.localizers.TwoWheelOdometryLocalizer
import com.atomicrobotics.cflib.opmodes.TeleOpMode
import org.firstinspires.ftc.teamcode.controls.ExampleControls
import org.firstinspires.ftc.teamcode.drive.CompetitionMecanumDriveConstants
import org.firstinspires.ftc.teamcode.localizers.CompetitionOdometryConstants
import org.firstinspires.ftc.teamcode.mechanisms.Arm
import org.firstinspires.ftc.teamcode.mechanisms.Claw
import org.firstinspires.ftc.teamcode.mechanisms.Lift
import org.firstinspires.ftc.teamcode.routines.Routines
import org.firstinspires.ftc.teamcode.trajectoryFactory.CompetitionTrajectoryFactory

/**
 * This class is an example of how you can create an TeleOp OpMode. Everything is handled by the
 * TeleOpMode parent class, so all you have to do is pass in the constructor parameters.
 */

@TeleOp(name = "Testing TeleOp OpMode", group="Testing")
@Disabled
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