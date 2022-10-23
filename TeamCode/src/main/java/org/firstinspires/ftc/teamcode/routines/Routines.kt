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

import com.atomicrobotics.cflib.Command
import com.atomicrobotics.cflib.Constants.drive
import com.atomicrobotics.cflib.driving.localizers.TwoWheelOdometryLocalizer
import com.atomicrobotics.cflib.example.mechanisms.Claw
import com.atomicrobotics.cflib.example.mechanisms.Lift
import com.atomicrobotics.cflib.parallel
import com.atomicrobotics.cflib.sequential
import com.atomicrobotics.cflib.utilCommands.TelemetryCommand
import org.firstinspires.ftc.teamcode.trajectoryFactory.CompetitionTrajectoryFactory

/**
 * This class is an example of how to create routines. Routines are essentially just groups of
 * commands that can be run either one at a time (sequentially) or all at once (in parallel).
 */
object Routines {

    val initializationRoutine: Command
        get() = parallel {
            +TelemetryCommand(999.9, "Estimated Position") { drive.localizer.poseEstimate.toString() }
            +TelemetryCommand(999.9, "Parallel Encoder") { (drive.localizer as TwoWheelOdometryLocalizer).parallelEncoder.currentPosition.toString() }
            +TelemetryCommand(999.9, "Perpendicular Encoder") { (drive.localizer as TwoWheelOdometryLocalizer).perpendicularEncoder.currentPosition.toString() }
        }

    val leftMainRoutine: Command
        get() = sequential {
            +drive.followTrajectory(CompetitionTrajectoryFactory.leftStartToLowJunction)
            // Score the preloaded cone onto the low junction
            +parallel {
                // Lower the lift to pick up the next element
                +drive.followTrajectory(CompetitionTrajectoryFactory.leftLowJunctionToTerminal)
            }
        }

    val teleOpStartRoutine: Command
        get() = parallel {
            +Lift.toLow
            +Claw.open
        }
}