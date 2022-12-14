package org.firstinspires.ftc.teamcode.routines

import com.atomicrobotics.cflib.*
import com.atomicrobotics.cflib.subsystems.DisplayRobot
import com.atomicrobotics.cflib.utilCommands.ConditionalCommand
import com.atomicrobotics.cflib.utilCommands.Delay
import com.atomicrobotics.cflib.utilCommands.TelemetryCommand
import org.firstinspires.ftc.teamcode.mechanisms.*
import org.firstinspires.ftc.teamcode.trajectoryFactory.CompetitionTrajectoryFactory
import org.firstinspires.ftc.teamcode.trajectoryFactory.NewTrajectoryFactory
import com.atomicrobotics.cflib.Constants.drive

object NewRoutines {
    val threePlusOne : Command
        get() = parallel {
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
                    +Delay(0.5)
                    +Arm.toHighJunction
                }
            }
            +Claw.open
        }

    val threeConeStack : Command
        get() = sequential {
            +parallel {
                +Arm.toForward
                +sequential {
                    +Delay(0.5)
                    +Lift.toLevel5
                }
                +Constants.drive.followTrajectory(NewTrajectoryFactory.startToHighJunction)
            }
            +Claw.close
            +parallel {
                +sequential {
                    +Delay(0.5)
                    +parallel {
                        +Constants.drive.followTrajectory(NewTrajectoryFactory.stackToHighJunction)
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
                +Constants.drive.followTrajectory(NewTrajectoryFactory.highJunctionToStack)
            }
            +Claw.close
            +parallel {
                +sequential {
                    +Delay(0.5)
                    +parallel {
                        +Constants.drive.followTrajectory(NewTrajectoryFactory.stackToHighJunction)
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
                    +Lift.toLevel3
                }
                +Constants.drive.followTrajectory(NewTrajectoryFactory.highJunctionToStack)
            }
            +Claw.close
            +parallel {
                +sequential {
                    +Delay(0.5)
                    +parallel {
                        +Constants.drive.followTrajectory(NewTrajectoryFactory.stackToHighJunction)
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
            +ConditionalCommand({ OpenCVWebcam.detectedColor == PowerPlayPipeline.SleeveColor.CYAN }, { CommandScheduler.scheduleCommand(
                drive.followTrajectory(CompetitionTrajectoryFactory.highJunctionToCyanResult)) })
            +ConditionalCommand({ OpenCVWebcam.detectedColor == PowerPlayPipeline.SleeveColor.MAGENTA }, { CommandScheduler.scheduleCommand(
                drive.followTrajectory(CompetitionTrajectoryFactory.highJunctionToMagentaResult)) })
            +ConditionalCommand({ OpenCVWebcam.detectedColor == PowerPlayPipeline.SleeveColor.YELLOW }, { CommandScheduler.scheduleCommand(
                drive.followTrajectory(CompetitionTrajectoryFactory.highJunctionToYellowResult)) })
            +ConditionalCommand({ OpenCVWebcam.detectedColor == PowerPlayPipeline.SleeveColor.UNDETECTED }, { CommandScheduler.scheduleCommand(
                drive.followTrajectory(CompetitionTrajectoryFactory.highJunctionToCyanResult)) })
        }
}