package org.firstinspires.ftc.teamcode.opmodes.autonomous

import com.atomicrobotics.unused.Constants
import com.atomicrobotics.unused.driving.drivers.MecanumDrive
import com.atomicrobotics.unused.driving.localizers.TwoWheelOdometryLocalizer
import com.atomicrobotics.unused.opmodes.AutonomousOpMode
import com.atomicrobotics.unused.subsystems.DisplayRobot
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.drive.CompetitionMecanumDriveConstants
import com.atomicrobotics.unused.trajectories.localizers.CompetitionOdometryConstants
import org.firstinspires.ftc.teamcode.trajectoryFactory.CompetitionTrajectoryFactory

@Autonomous
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