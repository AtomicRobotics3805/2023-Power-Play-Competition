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
import com.atomicrobotics.cflib.driving.localizers.TwoWheelOdometryLocalizer
import com.atomicrobotics.cflib.utilCommands.ConditionalCommand
import com.atomicrobotics.cflib.utilCommands.TelemetryCommand
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

    // Color sensor value


    val stackScoreRoutine: Command
        get() = sequential {
            // Drive to low junction
            +parallel {
                +drive.followTrajectory(CompetitionTrajectoryFactory.leftComplexStartToLowJunction)
                +Lift.toHigh
                +Arm.toMiddle
            }
            // Score preload
            +Claw.open

            // Drive to signal
            +parallel {
                +drive.followTrajectory(CompetitionTrajectoryFactory.leftComplexLowJunctionToSignal)
                +sequential {
                    +parallel {
                        +Claw.close
                        +Arm.toForward
                    }
                    +Lift.toLow
                    +Claw.open
                }
            }
            // Detect signal
            +ColorSensor.detect

            // CYCLE 1:
            // Drive to cone stack
            +parallel {
                +drive.followTrajectory(CompetitionTrajectoryFactory.leftSignalToConeStack)
                +Arm.toForward
                +Lift.toLevel5
            }
            // Pick up cone
            +Claw.close
            +sequential {
                // Lift cone
                +Lift.toHigh
                // Drive away
                parallel {
                    +drive.followTrajectory(CompetitionTrajectoryFactory.coneStackToHighJunction)
                    +Arm.toMiddle
                }
            }
            // Score cone
            +Claw.open

            // CYCLE 2:
            // Drive to cone stack
            +parallel {
                +drive.followTrajectory(CompetitionTrajectoryFactory.highJunctionToConeStack)
                +Arm.toForward
                +Lift.toLevel4
            }
            // Pick up cone
            +Claw.close
            +sequential {
                // Lift cone
                +Lift.toHigh
                // Drive away
                parallel {
                    +drive.followTrajectory(CompetitionTrajectoryFactory.coneStackToHighJunction)
                    +Arm.toMiddle
                }
            }
            // Score cone
            +Claw.open

            // CYCLE 3:
            // Drive to cone stack
            +parallel {
                +drive.followTrajectory(CompetitionTrajectoryFactory.highJunctionToConeStack)
                +Arm.toForward
                +Lift.toLevel3
            }
            // Pick up cone
            +Claw.close
            +sequential {
                // Lift cone
                +Lift.toHigh
                // Drive away
                parallel {
                    +drive.followTrajectory(CompetitionTrajectoryFactory.coneStackToHighJunction)
                    +Arm.toMiddle
                }
            }
            // Score cone
            +Claw.open

            // CYCLE 4:
            // Drive to cone stack
            +parallel {
                +drive.followTrajectory(CompetitionTrajectoryFactory.highJunctionToConeStack)
                +Arm.toForward
                +Lift.toLevel2
            }
            // Pick up cone
            +Claw.close
            +sequential {
                // Lift cone
                +Lift.toHigh
                // Drive away
                parallel {
                    +drive.followTrajectory(CompetitionTrajectoryFactory.coneStackToHighJunction)
                    +Arm.toMiddle
                }
            }
            // Score cone
            +Claw.open

            // CYCLE 5:
            // Drive to cone stack
            +parallel {
                +drive.followTrajectory(CompetitionTrajectoryFactory.highJunctionToConeStack)
                +Arm.toForward
                +Lift.toIntake
            }
            // Pick up cone
            +Claw.close
            +sequential {
                // Lift cone
                +Lift.toHigh
                // Drive away
                parallel {
                    +drive.followTrajectory(CompetitionTrajectoryFactory.coneStackToHighJunction)
                    +Arm.toMiddle
                }
            }
            // Score cone
            +Claw.open
            + sequential {
                +Lift.toIntake
                +drive.followTrajectory(CompetitionTrajectoryFactory.highJunctionToConeStack)
                +Claw.close
            }
            // Park
            +ConditionalCommand({ ColorSensor.detectedColor == ColorSensor.SleeveColor.RED }, { CommandScheduler.scheduleCommand(
                redRoutine)})
        }

    val blueRoutine: Command
        get() = sequential {

        }

    val redRoutine: Command
        get() = parallel {
            +drive.followTrajectory(CompetitionTrajectoryFactory.leftSignalResultRed)
        }
}