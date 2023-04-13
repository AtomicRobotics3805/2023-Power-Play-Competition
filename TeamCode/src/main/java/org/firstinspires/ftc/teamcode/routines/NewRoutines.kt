package org.firstinspires.ftc.teamcode.routines

import com.atomicrobotics.unused.*
import com.atomicrobotics.unused.subsystems.DisplayRobot
import com.atomicrobotics.unused.utilCommands.ConditionalCommand
import com.atomicrobotics.unused.utilCommands.Delay
import com.atomicrobotics.unused.utilCommands.TelemetryCommand
import org.firstinspires.ftc.teamcode.mechanisms.*
import org.firstinspires.ftc.teamcode.trajectoryFactory.NewTrajectoryFactory
import com.atomicrobotics.unused.Constants.drive
import com.atomicrobotics.unused.utilCommands.CustomCommand
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

    val stackCycleToCenterJunction : Command // OPMODE ROUTINE
        get() = parallel {
            +CustomCommand(_start = { timer.reset() })
            +TelemetryCommand(30.0, "Runtime") { timer.time().toString() }
            +sequential {
                +OpenCVWebcam.detect
                +preloadScoreCenter
                +cycleStackCenter
            }
            +TelemetryCommand(30.0, "Detected Color") { OpenCVWebcam.detectedColor.toString() }
            +DisplayRobot(14.5, 15.0)
        }

    val preloadScoreCenter : Command
        get() = sequential {
            +parallel {
                +Claw.close
                +Arm.toForward
                +drive.followTrajectory(NewTrajectoryFactory.startToCenterHighJunction)
                +Lift.toHigh
                +sequential {
                    +Delay(1.0)
                    +Arm.toHighJunction
                }
            }
            +score
        }

    val cycleStackCenter : Command
        get() = sequential {
            +parallel {
                +Arm.toForward
                +sequential {
                    +Delay(0.5)
                    +Lift.toLevel5
                }
                +drive.followTrajectory(NewTrajectoryFactory.startCenterHighJunctionToStack)
            }
            +Claw.close
            +parallel {
                +sequential {
                    +Delay(0.5)
                    +parallel {
                        +drive.followTrajectory(NewTrajectoryFactory.stackToCenterHighJunction)
                        +Arm.toOtherHighJunction
                    }
                }
                +Lift.toHigh
            }
            +score
            +parallel {
                +Arm.toForward
                +sequential {
                    +Delay(0.5)
                    +Lift.toLevel4
                }
                +drive.followTrajectory(NewTrajectoryFactory.centerHighJunctionToStack)
            }
            +Claw.close
            +parallel {
                +sequential {
                    +Delay(0.5)
                    +parallel {
                        +drive.followTrajectory(NewTrajectoryFactory.stackToCenterHighJunction)
                        +Arm.toOtherHighJunction
                    }
                }
                +Lift.toHigh
            }
            +score
            +parallel {
                +Arm.toForward
                +sequential {
                    +Delay(1.0)
                    +Lift.toIntake
                }
                +centerHighJunctionToSignalResult
            }
            +CustomCommand(_start = { lastTime = timer.seconds() })
            +TelemetryCommand(30.0, "Final Time") { lastTime.toString() }
        }

    val centerHighJunctionToSignalResult: Command
        get() = sequential {
            +ConditionalCommand({ OpenCVWebcam.detectedColor == PowerPlayPipeline.SleeveColor.CYAN }, { CommandScheduler.scheduleCommand(
                drive.followTrajectory(NewTrajectoryFactory.centerHighJunctionToCyanResult)) })
            +ConditionalCommand({ OpenCVWebcam.detectedColor == PowerPlayPipeline.SleeveColor.MAGENTA }, { CommandScheduler.scheduleCommand(
                drive.followTrajectory(NewTrajectoryFactory.centerHighJunctionToMagentaResult)) })
            +ConditionalCommand({ OpenCVWebcam.detectedColor == PowerPlayPipeline.SleeveColor.YELLOW }, { CommandScheduler.scheduleCommand(
                drive.followTrajectory(NewTrajectoryFactory.centerHighJunctionToYellowResult)) })
            +ConditionalCommand({ OpenCVWebcam.detectedColor == PowerPlayPipeline.SleeveColor.UNDETECTED }, { CommandScheduler.scheduleCommand(
                drive.followTrajectory(NewTrajectoryFactory.centerHighJunctionToCyanResult)) })
        }

    val preloadToStack : Command
        get() = sequential {
            +parallel {
                +Claw.close
                +Arm.toForward
                +drive.followTrajectory(NewTrajectoryFactory.startToHighJunction)
                +Lift.toHigh
                +sequential {
                    +Delay(1.0)
                    +Arm.toRight
                }
            }
            +score
        }

    val threeConeStack : Command
        get() = sequential {
            +parallel {
                +Arm.toForward
                +sequential {
                    +Delay(0.5)
                    +Lift.toLevel5
                }
                +drive.followTrajectory(NewTrajectoryFactory.startHighJunctionToStack)
            }
            +Claw.close
            +parallel {
                +sequential {
                    +Delay(0.5)
                    +parallel {
                        +drive.followTrajectory(NewTrajectoryFactory.stackToHighJunction)
                        +Arm.toHighJunction
                    }
                }
                +Lift.toHigh
            }
            +score
            +parallel {
                +Arm.toForward
                +sequential {
                    +Delay(0.5)
                    +Lift.toLevel4
                }
                +drive.followTrajectory(NewTrajectoryFactory.highJunctionToStack)
            }
            +Claw.close
            +parallel {
                +sequential {
                    +Delay(0.5)
                    +parallel {
                        +drive.followTrajectory(NewTrajectoryFactory.stackToHighJunction)
                        +Arm.toHighJunction
                    }
                }
                +Lift.toHigh
            }
            +score
            +parallel {
                +Arm.toForward
                +sequential {
                    +Delay(0.5)
                    +Lift.toLevel3
                }
                +drive.followTrajectory(NewTrajectoryFactory.highJunctionToStack)
            }
            +Claw.close
            +parallel {
                +sequential {
                    +Delay(0.5)
                    +parallel {
                        +drive.followTrajectory(NewTrajectoryFactory.stackToHighJunction)
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
                drive.followTrajectory(NewTrajectoryFactory.highJunctionToCyanResult)) })
            +ConditionalCommand({ OpenCVWebcam.detectedColor == PowerPlayPipeline.SleeveColor.MAGENTA }, { CommandScheduler.scheduleCommand(
                drive.followTrajectory(NewTrajectoryFactory.highJunctionToMagentaResult)) })
            +ConditionalCommand({ OpenCVWebcam.detectedColor == PowerPlayPipeline.SleeveColor.YELLOW }, { CommandScheduler.scheduleCommand(
                drive.followTrajectory(NewTrajectoryFactory.highJunctionToYellowResult)) })
            +ConditionalCommand({ OpenCVWebcam.detectedColor == PowerPlayPipeline.SleeveColor.UNDETECTED }, { CommandScheduler.scheduleCommand(
                drive.followTrajectory(NewTrajectoryFactory.highJunctionToCyanResult)) })
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