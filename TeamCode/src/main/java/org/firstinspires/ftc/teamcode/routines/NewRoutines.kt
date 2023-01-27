package org.firstinspires.ftc.teamcode.routines

import com.atomicrobotics.cflib.*
import com.atomicrobotics.cflib.subsystems.DisplayRobot
import com.atomicrobotics.cflib.utilCommands.ConditionalCommand
import com.atomicrobotics.cflib.utilCommands.Delay
import com.atomicrobotics.cflib.utilCommands.TelemetryCommand
import org.firstinspires.ftc.teamcode.mechanisms.*
import org.firstinspires.ftc.teamcode.trajectoryFactory.CompetitionTrajectoryFactory
import com.atomicrobotics.cflib.Constants.drive
import com.atomicrobotics.cflib.utilCommands.CustomCommand
import com.qualcomm.robotcore.util.ElapsedTime

object NewRoutines {

    var timer = ElapsedTime()
    var lastTime = 0.0

    val threePlusOne : Command
        get() = parallel {
            +CustomCommand(_start = { timer.reset() })
            +TelemetryCommand(30.0, "Runtime") { timer.time().toString() }
            +sequential {
                +OpenCVWebcam.detect
                +preloadToStack
                +threeConeStack
            }
            +TelemetryCommand(30.0, "Detected Color") { OpenCVWebcam.detectedColor.toString() }
            +DisplayRobot(14.5, 15.0)
        }

    val preloadToStack : Command
        get() = sequential {
            +parallel {
                +Claw.close
                +Arm.toForward
                +drive.followTrajectory(CompetitionTrajectoryFactory.startToHighJunction)
                +Lift.toHigh
                +sequential {
                    +Delay(1.0)
                    +Arm.toHighJunction
                }
            }
            +score
        }

    val threeConeStack : Command
        get() = sequential {
            +parallel {
                +Arm.toForward
                +sequential {
                    +Delay(1.0)
                    +Lift.toLevel5
                }
                +drive.followTrajectory(CompetitionTrajectoryFactory.startHighJunctionToStack)
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
            +score
            +parallel {
                +Arm.toForward
                +sequential {
                    +Delay(1.0)
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
            +score
            +parallel {
                +Arm.toForward
                +sequential {
                    +Delay(1.0)
                    +Lift.toLevel3
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
            +score
            +parallel {
                +Arm.toForward
                +sequential {
                    +Delay(1.5)
                    +Lift.toIntake
                }
                +highJunctionToSignalResult
            }
            +CustomCommand(_start = { lastTime = timer.seconds() })
            +TelemetryCommand(30.0, "Final Time") { lastTime.toString() }
        }

    val highJunctionToSignalResult: Command
        get() = sequential {
            +ConditionalCommand({ OpenCVWebcam.detectedColor == PowerPlayPipeline.SleeveColor.CYAN }, { CommandScheduler.scheduleCommand(
                drive.followTrajectory(CompetitionTrajectoryFactory.highJunctionToCyanResult)) })
            +ConditionalCommand({ OpenCVWebcam.detectedColor == PowerPlayPipeline.SleeveColor.MAGENTA }, { CommandScheduler.scheduleCommand(
                drive.followTrajectory(CompetitionTrajectoryFactory.highJunctionToMagentaResult)) })
            +ConditionalCommand({ OpenCVWebcam.detectedColor == PowerPlayPipeline.SleeveColor.YELLOW }, { CommandScheduler.scheduleCommand(
                drive.followTrajectory(CompetitionTrajectoryFactory.highJunctionToYellowResult)) })
            +ConditionalCommand({ OpenCVWebcam.detectedColor == PowerPlayPipeline.SleeveColor.UNDETECTED }, { CommandScheduler.scheduleCommand(
                drive.followTrajectory(CompetitionTrajectoryFactory.highJunctionToCyanResult)) })
        }

    val score: Command
        get() = sequential {
            +Delay(0.2)
            +parallel {
                +Claw.open
                +Lift.slightlyLower
            }
        }
}