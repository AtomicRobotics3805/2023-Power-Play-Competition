package org.firstinspires.ftc.teamcode.opmodes.autonomous.newdeadwheels

import com.atomicrobotics.cflib.Constants
import com.atomicrobotics.cflib.driving.drivers.MecanumDrive
import com.atomicrobotics.cflib.driving.localizers.TwoWheelOdometryLocalizer
import com.atomicrobotics.cflib.opmodes.AutonomousOpMode
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.drive.CompetitionMecanumDriveConstants
import org.firstinspires.ftc.teamcode.localizers.CompetitionOdometryConstants
import org.firstinspires.ftc.teamcode.mechanisms.Arm
import org.firstinspires.ftc.teamcode.mechanisms.Claw
import org.firstinspires.ftc.teamcode.mechanisms.Lift
import org.firstinspires.ftc.teamcode.mechanisms.OpenCVWebcam
import org.firstinspires.ftc.teamcode.routines.NewRoutines
import org.firstinspires.ftc.teamcode.trajectoryFactory.CompetitionTrajectoryFactory

@Autonomous(name = "3+1 and Park (Driver's Right)", group = "competition")
class ThreePlusOneRight : AutonomousOpMode(
    Constants.Side.RIGHT,
    CompetitionTrajectoryFactory,
    { NewRoutines.threePlusOne },
    null,
    MecanumDrive(
        CompetitionMecanumDriveConstants,
        TwoWheelOdometryLocalizer(CompetitionOdometryConstants())
    ) { CompetitionTrajectoryFactory.startPoseLeft },
    Arm,
    Claw,
    Lift,
    OpenCVWebcam
)