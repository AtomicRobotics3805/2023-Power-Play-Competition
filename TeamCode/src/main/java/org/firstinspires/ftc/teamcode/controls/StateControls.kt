package org.firstinspires.ftc.teamcode.controls

import com.atomicrobotics.cflib.Command
import com.atomicrobotics.cflib.CommandScheduler
import com.atomicrobotics.cflib.Constants
import com.atomicrobotics.cflib.controls.Controls
import com.atomicrobotics.cflib.parallel
import com.atomicrobotics.cflib.utilCommands.CustomCommand
import org.firstinspires.ftc.teamcode.mechanisms.Arm
import org.firstinspires.ftc.teamcode.mechanisms.Claw
import org.firstinspires.ftc.teamcode.mechanisms.ConeRighter
import org.firstinspires.ftc.teamcode.mechanisms.Lift

object StateControls : Controls(Constants.opMode.gamepad1) {
    override fun registerCommands() {
        // Speed switching: right bumper sets it to speed[2] (slowest), releasing sets to medium; left sets it to speed[0] (fastest), releasing sets to medium
        gamepad1.rightBumper.pressedCommand = { Constants.drive.setSpeed(2) }
        gamepad1.rightBumper.releasedCommand = { Constants.drive.setSpeed(1) }
        gamepad2.leftBumper.pressedCommand = { Constants.drive.setSpeed(0) }
        gamepad2.leftBumper.releasedCommand = { Constants.drive.setSpeed(1) }

        // gamepad1.b.pressedCommand = { reset robot orientation for field centric }
        // set tolerance of trigger
        gamepad2.rightTrigger.tolerance = 0.5f
        gamepad2.rightTrigger.pressedCommand = { CustomCommand(_start = { gamepad2.shiftActivated = true }) }
        gamepad2.rightTrigger.releasedCommand = { CustomCommand(_start = { gamepad2.shiftActivated = false }) }
        gamepad2.dpadUp.secondaryPressedCommand = { Lift.start }
        gamepad2.dpadUp.secondaryReleasedCommand = { Lift.stop }
        gamepad2.dpadDown.secondaryPressedCommand = { Lift.reverse }
        gamepad2.dpadDown.secondaryReleasedCommand = { Lift.stop }

        gamepad2.leftBumper.pressedCommand = { Claw.open }
        gamepad1.a.pressedCommand = { Claw.open }
        gamepad2.rightBumper.pressedCommand = { Claw.close }
        gamepad1.x.pressedCommand = { Claw.close }

        gamepad1.leftTrigger.tolerance = 0.75f
        gamepad1.leftTrigger.pressedCommand = { ConeRighter.extend }
        gamepad1.leftTrigger.releasedCommand = { ConeRighter.retract }
        gamepad2.a.pressedCommand = { parallel {
            +Claw.open
            +Arm.toForward
        } }
    }
}