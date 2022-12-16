package org.firstinspires.ftc.teamcode.opmodes.teleop

import com.atomicrobotics.cflib.driving.drivers.MecanumDrive
import com.atomicrobotics.cflib.driving.localizers.TwoWheelOdometryLocalizer
import com.atomicrobotics.cflib.opmodes.TeleOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.controls.ExampleControls
import org.firstinspires.ftc.teamcode.drive.CompetitionMecanumDriveConstants
import org.firstinspires.ftc.teamcode.localizers.CompetitionOdometryConstants
import org.firstinspires.ftc.teamcode.trajectoryFactory.NewTrajectoryFactory

@TeleOp(name="State TeleOp")
class StateTeleOp : TeleOpMode(
    ExampleControls,
    drive = MecanumDrive (
        CompetitionMecanumDriveConstants,
        TwoWheelOdometryLocalizer(CompetitionOdometryConstants())
    ) { NewTrajectoryFactory.startPoseLeft }
)