/*
    Copyright (c) 2022 Atomic Robotics (https://atomicrobotics3805.org)

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see https://www.gnu.org/licenses/.
*/
package org.firstinspires.ftc.teamcode.routines

import com.atomicrobotics.cflib.*
import com.atomicrobotics.cflib.Constants.drive
import com.atomicrobotics.cflib.utilCommands.ConditionalCommand
import com.atomicrobotics.cflib.utilCommands.Delay
import org.firstinspires.ftc.teamcode.mechanisms.Arm
import org.firstinspires.ftc.teamcode.mechanisms.Claw
import org.firstinspires.ftc.teamcode.mechanisms.ColorSensor
import org.firstinspires.ftc.teamcode.mechanisms.Lift
import org.firstinspires.ftc.teamcode.trajectoryFactory.CompetitionTrajectoryFactory

/**
 * This class is an example of how to create routines. Routines are essentially just groups of
 * commands that can be run either one at a time (sequentially) or all at once (in parallel).
 */
object Routines {

    val initializationRoutine: Command
        get() = sequential {
//            +TelemetryCommand(999.9, "Estimated Position") { drive.localizer.poseEstimate.toString() }
//            +TelemetryCommand(999.9, "Parallel Encoder") { (drive.localizer as TwoWheelOdometryLocalizer).parallelEncoder.currentPosition.toString() }
//            +TelemetryCommand(999.9, "Perpendicular Encoder") { (drive.localizer as TwoWheelOdometryLocalizer).perpendicularEncoder.currentPosition.toString() }
            +Claw.close
            +Arm.toForward
        }

    val lowJunctionScoreParkInTerminal: Command
        get() = sequential {
            +parallel {
                +Lift.toLow
                +drive.followTrajectory(CompetitionTrajectoryFactory.awayStartToLowJunction)
            }
            // Score the preloaded cone onto the low junction
            +Claw.open
            +Delay(0.25)
            +parallel {
                +drive.followTrajectory(CompetitionTrajectoryFactory.awayLowJunctionToTerminal)
                +Lift.toIntake
            }
        }

    val lowJunctionScoreParkInSignalZoneRed: Command
        get() = sequential {
            +parallel {
                +Lift.toLow
                +drive.followTrajectory(CompetitionTrajectoryFactory.awayStartToLowJunction)
            }
            +Claw.open
            +Delay(0.25)
            +parallel {
                +drive.followTrajectory(CompetitionTrajectoryFactory.awayLowJunctionToAwaySignalRed)
                +Lift.toIntake
            }
            +ColorSensor.detect
            +ConditionalCommand({ ColorSensor.color == ColorSensor.SleeveColor.BLUE }, { CommandScheduler.scheduleCommand(blueRoutine) })
            +ConditionalCommand({ ColorSensor.color == ColorSensor.SleeveColor.RED }, { CommandScheduler.scheduleCommand(redRoutine) })
            +ConditionalCommand({ ColorSensor.color == ColorSensor.SleeveColor.GREEN }, { CommandScheduler.scheduleCommand(greenRoutine) })
        }


    // Color sensor value

    val teleOpStartRoutine: Command
        get() = sequential {

        }

    val blueRoutine: Command
        get() = sequential {
            +drive.followTrajectory(CompetitionTrajectoryFactory.awaySignalResultBlue)
        }

    val redRoutine: Command
        get() = sequential {
            +drive.followTrajectory(CompetitionTrajectoryFactory.e5SignalResultRed)
        }

    val greenRoutine: Command
        get() = sequential {
            +drive.followTrajectory(CompetitionTrajectoryFactory.e5SignalResultGreen)
        }
}