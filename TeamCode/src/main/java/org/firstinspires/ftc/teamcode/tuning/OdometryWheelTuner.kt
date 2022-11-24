@file:Suppress("PropertyName")

package org.firstinspires.ftc.teamcode.tuning

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.util.epsilonEquals
import com.atomicrobotics.cflib.Command
import com.atomicrobotics.cflib.CommandScheduler
import com.atomicrobotics.cflib.Constants
import com.atomicrobotics.cflib.Constants.drive
import com.atomicrobotics.cflib.Constants.opMode
import com.atomicrobotics.cflib.driving.Turn
import com.atomicrobotics.cflib.driving.drivers.MecanumDrive
import com.atomicrobotics.cflib.driving.localizers.TwoWheelOdometryLocalizer
import com.atomicrobotics.cflib.opmodes.AutonomousOpMode
import com.atomicrobotics.cflib.utilCommands.TelemetryCommand
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.drive.CompetitionMecanumDriveConstants
import org.firstinspires.ftc.teamcode.localizers.CompetitionOdometryConstants
import kotlin.math.PI

@Autonomous
@Config
class OdometryWheelTuner : AutonomousOpMode(
    Constants.Color.UNKNOWN,
    null,
    { TuneWheels() },
    null,
    MecanumDrive(
        CompetitionMecanumDriveConstants,
        TwoWheelOdometryLocalizer(CompetitionOdometryConstants())
    )
) {

    @JvmField
    var TURN_NUMBER = 10.0

    class TuneWheels: Command() {
        override val _isDone = false

        var timer: ElapsedTime? = null

        override fun start() {
            // tells robot to start turning
            CommandScheduler.scheduleCommand(drive.turn(
                (opMode as OdometryWheelTuner).TURN_NUMBER * 2 * PI, Turn.TurnType.RELATIVE
                ))
        }

        override fun execute() {
            // if the robot is not turning, starts a timer
            if (drive.poseVelocity != null && drive.poseVelocity!!.heading epsilonEquals 0.0) {
                timer = ElapsedTime()
            }
            if (timer != null && timer!!.seconds() > 0.1) {
                // if the robot is still not turning, decides that the turning has finished
                if (drive.poseVelocity != null && drive.poseVelocity!!.heading epsilonEquals 0.0) {
                    // calculates the amount that the robot turned, since it's probably not the exact target amount
                    val heading = drive.poseEstimate.heading
                    val amountTurned = (opMode as OdometryWheelTuner).TURN_NUMBER + heading / (2 * PI)
                    // calculates the positions of the wheels
                    val distanceTraveled = (drive.localizer as TwoWheelOdometryLocalizer)
                        .getWheelPositions()
                    val parallelY = -distanceTraveled[0] / amountTurned
                    val perpendicularX = -distanceTraveled[1] / amountTurned
                    // sets the constants and displays them on the telemetry
                    ((drive.localizer as TwoWheelOdometryLocalizer).
                    constants as CompetitionOdometryConstants)._PARALLEL_Y = parallelY
                    ((drive.localizer as TwoWheelOdometryLocalizer).
                    constants as CompetitionOdometryConstants)._PERPENDICULAR_X = perpendicularX
                    CommandScheduler.scheduleCommand(TelemetryCommand(9999.0, "Parallel Y", parallelY.toString()))
                    CommandScheduler.scheduleCommand(TelemetryCommand(9999.0, "Perpendicular X", perpendicularX.toString()))
                    // marks the command as finished
                    isDone = true
                }
                // if the robot has started turning, sets the timer to null again
                else {
                    timer = null
                }
            }
        }
    }
}
