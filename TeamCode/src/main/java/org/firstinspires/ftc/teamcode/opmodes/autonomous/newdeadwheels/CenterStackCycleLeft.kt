package org.firstinspires.ftc.teamcode.opmodes.autonomous.newdeadwheels

import com.atomicrobotics.unused.Constants
import com.atomicrobotics.unused.driving.drivers.MecanumDrive
import com.atomicrobotics.unused.driving.localizers.TwoWheelOdometryLocalizer
import com.atomicrobotics.unused.opmodes.AutonomousOpMode
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.drive.CompetitionMecanumDriveConstants
import com.atomicrobotics.unused.trajectories.localizers.CompetitionOdometryConstants
import org.firstinspires.ftc.teamcode.mechanisms.Arm
import org.firstinspires.ftc.teamcode.mechanisms.Claw
import org.firstinspires.ftc.teamcode.mechanisms.Lift
import org.firstinspires.ftc.teamcode.mechanisms.OpenCVWebcam
import org.firstinspires.ftc.teamcode.routines.NewRoutines
import org.firstinspires.ftc.teamcode.trajectoryFactory.NewTrajectoryFactory

@Autonomous(name = "Center High Junction Cycle (Driver's Left)")
class CenterStackCycleLeft : AutonomousOpMode(
    Constants.Side.LEFT,
    NewTrajectoryFactory,
    { NewRoutines.stackCycleToCenterJunction },
    null,
    MecanumDrive(
        CompetitionMecanumDriveConstants,
        TwoWheelOdometryLocalizer(CompetitionOdometryConstants())
    ) { NewTrajectoryFactory.startPoseLeft },
    Arm,
    Claw,
    Lift,
    OpenCVWebcam
)