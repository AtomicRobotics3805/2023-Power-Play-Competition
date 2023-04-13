package org.firstinspires.ftc.teamcode.opmodes.autonomous

import com.atomicrobotics.unused.Constants
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.atomicrobotics.unused.opmodes.AutonomousOpMode
import org.firstinspires.ftc.teamcode.drive.CompetitionMecanumDriveConstants
import com.atomicrobotics.unused.driving.drivers.MecanumDrive
import com.atomicrobotics.unused.driving.localizers.TwoWheelOdometryLocalizer
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.atomicrobotics.unused.trajectories.localizers.CompetitionOdometryConstants
import org.firstinspires.ftc.teamcode.mechanisms.Claw
import org.firstinspires.ftc.teamcode.mechanisms.Lift
import org.firstinspires.ftc.teamcode.mechanisms.Arm
import org.firstinspires.ftc.teamcode.mechanisms.ColorSensor
import org.firstinspires.ftc.teamcode.routines.Routines
import org.firstinspires.ftc.teamcode.trajectoryFactory.CompetitionTrajectoryFactory

@Disabled
@Autonomous(name = "Score Preload & Detect Signal (Driver's Right)")
class BasicSignalDetectionRight : AutonomousOpMode(
    Constants.Color.RED,
    CompetitionTrajectoryFactory,
    { Routines.lowJunctionScoreParkInSignalZoneRight },
    { Routines.initializationRoutine },
    MecanumDrive(
        CompetitionMecanumDriveConstants,
        TwoWheelOdometryLocalizer(CompetitionOdometryConstants())
    ) { CompetitionTrajectoryFactory.legalStartPose },
    Arm,
    Claw,
    Lift,
    ColorSensor
)