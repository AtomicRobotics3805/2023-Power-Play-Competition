package org.firstinspires.ftc.teamcode.opmodes

import com.atomicrobotics.cflib.Constants
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.atomicrobotics.cflib.opmodes.AutonomousOpMode
import org.firstinspires.ftc.teamcode.drive.CompetitionMecanumDriveConstants
import com.atomicrobotics.cflib.driving.drivers.MecanumDrive
import com.atomicrobotics.cflib.driving.localizers.TwoWheelOdometryLocalizer
import org.firstinspires.ftc.teamcode.localizers.CompetitionOdometryConstants
import org.firstinspires.ftc.teamcode.mechanisms.Claw
import org.firstinspires.ftc.teamcode.mechanisms.Lift
import org.firstinspires.ftc.teamcode.mechanisms.Arm
import org.firstinspires.ftc.teamcode.routines.Routines
import org.firstinspires.ftc.teamcode.trajectoryFactory.CompetitionTrajectoryFactory

@Autonomous(name = "Score Preload in Low Junction & Park in Terminal (RED)")
class LowJunctionPreloadScoreParkInTerminalREDV2 : AutonomousOpMode(
    Constants.Color.RED,
    CompetitionTrajectoryFactory,
    { Routines.lowJunctionScoreParkInTerminal },
    { Routines.initializationRoutine },
    MecanumDrive(
        CompetitionMecanumDriveConstants,
        TwoWheelOdometryLocalizer(CompetitionOdometryConstants())
    ) { CompetitionTrajectoryFactory.startPoseAway },
    Arm,
    Claw,
    Lift
)