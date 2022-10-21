package org.firstinspires.ftc.teamcode

import com.atomicrobotics.cflib.Constants
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.atomicrobotics.cflib.opmodes.AutonomousOpMode
import org.firstinspires.ftc.teamcode.CompetitionTrajectoryFactory
import org.firstinspires.ftc.teamcode.Routines
import org.firstinspires.ftc.teamcode.CompetitionMecanumDriveConstants
import com.atomicrobotics.cflib.driving.MecanumDriveConstants
import com.atomicrobotics.cflib.driving.drivers.MecanumDrive
import com.atomicrobotics.cflib.driving.localizers.Localizer
import com.atomicrobotics.cflib.driving.localizers.TwoWheelOdometryLocalizer
import org.firstinspires.ftc.teamcode.CompetitionOdometryConstants
import com.atomicrobotics.cflib.driving.localizers.TwoWheelOdometryConstants

@Autonomous(name = "Score Preload in Low Junction & Park in Terminal")
class LowJunctionPreloadScoreParkInTerminalV2 : AutonomousOpMode(
    Constants.Color.BLUE,
    CompetitionTrajectoryFactory,
    { Routines.leftMainRoutine },
    { Routines.initializationRoutine },
    MecanumDrive(
        CompetitionMecanumDriveConstants,
        TwoWheelOdometryLocalizer(CompetitionOdometryConstants())
    ) { CompetitionTrajectoryFactory.leftStartPose }
)