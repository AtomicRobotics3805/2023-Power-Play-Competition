package org.firstinspires.ftc.teamcode.opmodes.autonomous.backup

import com.atomicrobotics.cflib.Constants
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.atomicrobotics.cflib.opmodes.AutonomousOpMode
import org.firstinspires.ftc.teamcode.drive.CompetitionMecanumDriveConstants
import com.atomicrobotics.cflib.driving.drivers.MecanumDrive
import com.atomicrobotics.cflib.driving.localizers.TwoWheelOdometryLocalizer
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import org.firstinspires.ftc.teamcode.localizers.CompetitionOdometryConstants
import org.firstinspires.ftc.teamcode.mechanisms.*
import org.firstinspires.ftc.teamcode.routines.Routines
import org.firstinspires.ftc.teamcode.trajectoryFactory.CompetitionTrajectoryFactory

@Disabled
@Autonomous(name = "Score Preload & Detect Signal w/ Camera (Driver's left)")
class BetterSignalDetectionLeft : AutonomousOpMode(
    Constants.Color.BLUE,
    CompetitionTrajectoryFactory,
    { Routines.lowJunctionScoreParkInSignalZoneUsingCamera },
    { Routines.initializationRoutine },
    MecanumDrive(
        CompetitionMecanumDriveConstants,
        TwoWheelOdometryLocalizer(CompetitionOdometryConstants())
    ) { CompetitionTrajectoryFactory.centeredStartPose },
    Arm,
    Claw,
    Lift,
    OpenCVWebcam
)