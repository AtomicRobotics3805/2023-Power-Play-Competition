package org.firstinspires.ftc.teamcode.opmodes

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.atomicrobotics.cflib.Constants
import com.atomicrobotics.cflib.driving.drivers.MecanumDrive
import com.atomicrobotics.cflib.driving.localizers.TwoWheelOdometryLocalizer
import com.atomicrobotics.cflib.opmodes.AutonomousOpMode
import com.atomicrobotics.cflib.sequential
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.drive.CompetitionMecanumDriveConstants
import org.firstinspires.ftc.teamcode.localizers.CompetitionOdometryConstants
import org.firstinspires.ftc.teamcode.routines.Routines
import org.firstinspires.ftc.teamcode.trajectoryFactory.CompetitionTrajectoryFactory

@Autonomous(name = "Drive Strafe Test")
class DriveStrafeTest : AutonomousOpMode (
    Constants.Color.BLUE,
    CompetitionTrajectoryFactory,
    { sequential {
        +Constants.drive.followTrajectory(CompetitionTrajectoryFactory.forward)
        +Constants.drive.followTrajectory(CompetitionTrajectoryFactory.strafeRight)
    } },
    { Routines.initializationRoutine },
    MecanumDrive(
        CompetitionMecanumDriveConstants,
        TwoWheelOdometryLocalizer(CompetitionOdometryConstants())
    ) { Pose2d() }
)