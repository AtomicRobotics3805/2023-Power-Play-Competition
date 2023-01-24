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