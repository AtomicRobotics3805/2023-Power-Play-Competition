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
import com.atomicrobotics.cflib.subsystems.DisplayRobot
import com.atomicrobotics.cflib.utilCommands.ConditionalCommand
import com.atomicrobotics.cflib.utilCommands.Delay
import com.atomicrobotics.cflib.utilCommands.TelemetryCommand
import org.apache.commons.math3.analysis.function.Pow
import org.firstinspires.ftc.teamcode.mechanisms.*
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
        }


    val lowJunctionScoreParkInSignalZoneRight: Command
        get() = sequential {
            +parallel {
                +Claw.close
                +Arm.toForward
            }
            +Delay(1.0)
            +parallel {
                +Lift.toLow
                +drive.followTrajectory(CompetitionTrajectoryFactory.startToLowJunction)
            }
            +Claw.open
            +Delay(0.25)
            +parallel {
                +drive.followTrajectory(CompetitionTrajectoryFactory.lowJunctionToSignalRight)
                +Lift.toIntake
            }
            +ColorSensor.detect
            +ConditionalCommand({ ColorSensor.color == ColorSensor.SleeveColor.BLUE }, { CommandScheduler.scheduleCommand(blueRoutine) })
            +ConditionalCommand({ ColorSensor.color == ColorSensor.SleeveColor.RED }, { CommandScheduler.scheduleCommand(redRoutine) })
            +ConditionalCommand({ ColorSensor.color == ColorSensor.SleeveColor.GREEN }, { CommandScheduler.scheduleCommand(greenRoutine) })
        }

    val lowJunctionScoreParkInSignalZoneLeft: Command
        get() = sequential {
            +parallel {
                +Claw.close
                +Arm.toForward
            }
            +Delay(1.0)
            +parallel {
                +Lift.toLow
                +drive.followTrajectory(CompetitionTrajectoryFactory.startToLowJunction)
            }
            +Claw.open
            +Delay(0.25)
            +parallel {
                +drive.followTrajectory(CompetitionTrajectoryFactory.lowJunctionToSignalLeft)
                +Lift.toIntake
            }
            +ColorSensor.detect
            +ConditionalCommand({ ColorSensor.color == ColorSensor.SleeveColor.BLUE }, { CommandScheduler.scheduleCommand(blueRoutine) })
            +ConditionalCommand({ ColorSensor.color == ColorSensor.SleeveColor.RED }, { CommandScheduler.scheduleCommand(redRoutine) })
            +ConditionalCommand({ ColorSensor.color == ColorSensor.SleeveColor.GREEN }, { CommandScheduler.scheduleCommand(greenRoutine) })
        }

    val lowJunctionScoreParkInSignalZoneUsingCamera: Command
        get() = parallel {
            +sequential {
                +OpenCVWebcam.detect
                +parallel {
                    +Claw.close
                    +Arm.toForward
                    +Lift.toLow
                    +drive.followTrajectory(CompetitionTrajectoryFactory.centerStartToLowJunction)
                }
                +Claw.open
                +Lift.toIntake
                +ConditionalCommand({ OpenCVWebcam.detectedColor == PowerPlayPipeline.SleeveColor.CYAN }, { CommandScheduler.scheduleCommand(drive.followTrajectory(CompetitionTrajectoryFactory.lowJunctionToCyanResult)) })
                +ConditionalCommand({ OpenCVWebcam.detectedColor == PowerPlayPipeline.SleeveColor.MAGENTA }, { CommandScheduler.scheduleCommand(drive.followTrajectory(CompetitionTrajectoryFactory.lowJunctionToMagentaResult)) })
                +ConditionalCommand({ OpenCVWebcam.detectedColor == PowerPlayPipeline.SleeveColor.YELLOW }, { CommandScheduler.scheduleCommand(drive.followTrajectory(CompetitionTrajectoryFactory.lowJunctionToYellowResult)) })
            }
            +DisplayRobot(14.5, 15.0)
        }

    // Color sensor value

    val teleOpStartRoutine: Command
        get() = sequential {
            +DisplayRobot(14.5, 15.0)
        }

    val blueRoutine: Command
        get() = sequential {
            +drive.followTrajectory(CompetitionTrajectoryFactory.signalResultBlue)
        }

    val redRoutine: Command
        get() = sequential {
            +drive.followTrajectory(CompetitionTrajectoryFactory.signalResultRed)
        }

    val greenRoutine: Command
        get() = sequential {
            +drive.followTrajectory(CompetitionTrajectoryFactory.signalResultGreen)
        }

    val scorePreloadInHighJunctionToStartStackRoutine: Command
        get() = sequential {
            +parallel {
                +Claw.close
                +Arm.toForward
                +drive.followTrajectory(CompetitionTrajectoryFactory.centerStartToHighJunction)
                +Lift.toHigh
                +sequential {
                    +Delay(0.5)
                    +Arm.toHighJunction
                }
            }
            +Claw.open
        }

    val fiftyPointRoutine: Command
        get() = parallel {
            +sequential {
                +OpenCVWebcam.detect
                //+testRoutine
                +scorePreloadInHighJunctionToStartStackRoutine
                +stackRoutine
            }
            +TelemetryCommand(30.0, "Detected Color") { OpenCVWebcam.detectedColor.toString() }
            +DisplayRobot(14.5, 15.0)
        }

    val stackRoutine: Command
        get() = sequential {
            +parallel {
                +Arm.toForward
                +sequential {
                    +Delay(0.5)
                    +Lift.toLevel5
                }
                +drive.followTrajectory(CompetitionTrajectoryFactory.highJunctionToStack)
            }
            +Claw.close
            +parallel {
                +sequential {
                    +Delay(0.5)
                    +parallel {
                        +drive.followTrajectory(CompetitionTrajectoryFactory.stackToHighJunction)
                        +Arm.toHighJunction
                    }
                }
                +Lift.toHigh
            }
            +Claw.open
            +parallel {
                +Arm.toForward
                +sequential {
                    +Delay(0.5)
                    +Lift.toLevel4
                }
                +drive.followTrajectory(CompetitionTrajectoryFactory.highJunctionToStack)
            }
            +Claw.close
            +parallel {
                +sequential {
                    +Delay(0.5)
                    +parallel {
                        +drive.followTrajectory(CompetitionTrajectoryFactory.stackToHighJunction)
                        +Arm.toHighJunction
                    }
                }
                +Lift.toHigh
            }
            +Claw.open
            +parallel {
                +Arm.toForward
                +sequential {
                    +Delay(0.5)
                    +Lift.toIntake
                }
                +highJunctionToSignalResult
            }
        }

    val highJunctionToSignalResult: Command
        get() = sequential {
            //+ConditionalCommand({ColorSensor.detectedColor == ColorSensor.SleeveColor.BLUE}, {CommandScheduler.scheduleCommand(drive.followTrajectory(CompetitionTrajectoryFactory.highJunctionToMagentaResult))}, {CommandScheduler.scheduleCommand(ConditionalCommand({ColorSensor.detectedColor == ColorSensor.SleeveColor.GREEN}, {CommandScheduler.scheduleCommand(drive.followTrajectory(CompetitionTrajectoryFactory.highJunctionToCyanResult))}, {CommandScheduler.scheduleCommand(drive.followTrajectory(CompetitionTrajectoryFactory.highJunctionToYellowResult))}))})
            +ConditionalCommand({ OpenCVWebcam.detectedColor == PowerPlayPipeline.SleeveColor.CYAN }, { CommandScheduler.scheduleCommand(drive.followTrajectory(CompetitionTrajectoryFactory.highJunctionToCyanResult)) })
            +ConditionalCommand({ OpenCVWebcam.detectedColor == PowerPlayPipeline.SleeveColor.MAGENTA }, { CommandScheduler.scheduleCommand(drive.followTrajectory(CompetitionTrajectoryFactory.highJunctionToMagentaResult)) })
            +ConditionalCommand({ OpenCVWebcam.detectedColor == PowerPlayPipeline.SleeveColor.YELLOW }, { CommandScheduler.scheduleCommand(drive.followTrajectory(CompetitionTrajectoryFactory.highJunctionToYellowResult)) })
            +ConditionalCommand({ OpenCVWebcam.detectedColor == PowerPlayPipeline.SleeveColor.UNDETECTED }, { CommandScheduler.scheduleCommand(drive.followTrajectory(CompetitionTrajectoryFactory.highJunctionToCyanResult)) })
        }

    val twoConeStackCycleBlue: Command
        get() = sequential {
            +parallel {
                +Claw.open
                +Lift.toLevel5
                +Arm.toForward
            }
            +Claw.close
            +Delay(0.25)
            +parallel {
                +sequential {
                    +Delay(0.5)
                    +parallel {
                        +drive.followTrajectory(CompetitionTrajectoryFactory.stackToHighJunction)
                        +Arm.toRight
                    }
                }
                +Lift.toHigh
            }
            +Claw.open
            +Delay(0.25)
            +parallel {
                +Arm.toForward
                +drive.followTrajectory(CompetitionTrajectoryFactory.highJunctionToStack)
                +Lift.toLevel4
            }
            +Claw.close
            +Delay(0.25)
            +parallel {
                +sequential {
                    +Delay(0.5)
                    +parallel {
                        +drive.followTrajectory(CompetitionTrajectoryFactory.stackToHighJunction)
                        +Arm.toRight
                    }
                }
                +Lift.toHigh
            }
            +Claw.open
            +Delay(0.25)
            +parallel {
                +Arm.toForward
                +drive.followTrajectory(CompetitionTrajectoryFactory.highJunctionToStack)
            }
        }
}