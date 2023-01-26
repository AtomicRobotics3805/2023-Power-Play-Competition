package org.firstinspires.ftc.teamcode.opmodes.autonomous

import com.atomicrobotics.cflib.Constants
import com.atomicrobotics.cflib.driving.drivers.MecanumDrive
import com.atomicrobotics.cflib.driving.localizers.TwoWheelOdometryLocalizer
import com.atomicrobotics.cflib.opmodes.AutonomousOpMode
import com.atomicrobotics.cflib.subsystems.DisplayRobot
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import org.firstinspires.ftc.teamcode.drive.CompetitionMecanumDriveConstants
import org.firstinspires.ftc.teamcode.localizers.CompetitionOdometryConstants
import org.firstinspires.ftc.teamcode.trajectoryFactory.CompetitionTrajectoryFactory

@Autonomous
@Disabled
class DisplayRobotTest: AutonomousOpMode(
    Constants.Color.BLUE,
    CompetitionTrajectoryFactory,
    { DisplayRobot(14.5, 15.0) },
    null,
    MecanumDrive(
        CompetitionMecanumDriveConstants,
        TwoWheelOdometryLocalizer(CompetitionOdometryConstants())
    ) { CompetitionTrajectoryFactory.centeredStartPose },
)