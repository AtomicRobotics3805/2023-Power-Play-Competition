package org.firstinspires.ftc.teamcode.opmodes.autonomous

import com.atomicrobotics.unused.Constants
import com.atomicrobotics.unused.driving.drivers.MecanumDrive
import com.atomicrobotics.unused.driving.localizers.TwoWheelOdometryLocalizer
import com.atomicrobotics.unused.opmodes.AutonomousOpMode
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import org.firstinspires.ftc.teamcode.drive.CompetitionMecanumDriveConstants
import com.atomicrobotics.unused.trajectories.localizers.CompetitionOdometryConstants
import org.firstinspires.ftc.teamcode.mechanisms.*
import org.firstinspires.ftc.teamcode.routines.Routines
import org.firstinspires.ftc.teamcode.trajectoryFactory.CompetitionTrajectoryFactory

@Disabled
@Autonomous(name = "3-Cone Stack Cycle & Park (Driver's Right)")
class StackCycleRight : AutonomousOpMode(
    Constants.Color.RED,
    CompetitionTrajectoryFactory,
    { Routines.fiftyPointRoutine },
    null,
    MecanumDrive(
        CompetitionMecanumDriveConstants,
        TwoWheelOdometryLocalizer(CompetitionOdometryConstants())
    ) { CompetitionTrajectoryFactory.centeredStartPose },
    Arm,
    Claw,
    Lift,
    OpenCVWebcam
)