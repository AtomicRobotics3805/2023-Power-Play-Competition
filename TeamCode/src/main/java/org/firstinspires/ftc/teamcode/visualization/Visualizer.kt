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
package org.firstinspires.ftc.teamcode.visualization

import com.atomicrobotics.unused.Constants
import com.atomicrobotics.unused.driving.drivers.MecanumDrive
import com.atomicrobotics.unused.driving.localizers.TwoWheelOdometryLocalizer
import com.atomicrobotics.unused.sequential
import com.atomicrobotics.unused.visualization.MeepMeepRobot
import com.atomicrobotics.unused.visualization.MeepMeepVisualizer
import org.firstinspires.ftc.teamcode.drive.CompetitionMecanumDriveConstants
import com.atomicrobotics.unused.trajectories.localizers.CompetitionOdometryConstants
import org.firstinspires.ftc.teamcode.trajectoryFactory.NewTrajectoryFactory

fun main() {
    MeepMeepVisualizer.addRobot(MeepMeepRobot(
        MecanumDrive(
            CompetitionMecanumDriveConstants,
            TwoWheelOdometryLocalizer(CompetitionOdometryConstants())
        ) { NewTrajectoryFactory.preloadCenterHighJunctionLocationLeft },
        14.5,
        15.0,
        {
            sequential {
                +Constants.drive.followTrajectory(NewTrajectoryFactory.startToCenterHighJunction)
                +Constants.drive.followTrajectory(NewTrajectoryFactory.startCenterHighJunctionToStack)
            }
        },
        Constants.Side.RIGHT
    ))
    MeepMeepVisualizer.run(NewTrajectoryFactory)
}