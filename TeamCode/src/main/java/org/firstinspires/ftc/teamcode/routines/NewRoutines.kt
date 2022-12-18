package org.firstinspires.ftc.teamcode.routines

import com.atomicrobotics.cflib.*
import com.atomicrobotics.cflib.subsystems.DisplayRobot
import com.atomicrobotics.cflib.utilCommands.ConditionalCommand
import com.atomicrobotics.cflib.utilCommands.Delay
import com.atomicrobotics.cflib.utilCommands.TelemetryCommand
import org.firstinspires.ftc.teamcode.mechanisms.*
import org.firstinspires.ftc.teamcode.trajectoryFactory.NewTrajectoryFactory
import com.atomicrobotics.cflib.Constants.drive
import com.atomicrobotics.cflib.utilCommands.CustomCommand
import com.qualcomm.robotcore.util.ElapsedTime

object NewRoutines {

    var timer = ElapsedTime()
    var lastTime = 0.0

    val threePlusOneLeft : Command
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
                +drive.followTrajectory(NewTrajectoryFactory.startToHighJunction)
                +Lift.toHigh
                +sequential {
                    +Delay(1.0)
                    +Arm.toHighJunction
                }
            }
            +Delay(0.3)
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
            +Delay(0.4)
            +parallel {
                +Claw.open
                +Lift.slightlyLower
            }
        }
}