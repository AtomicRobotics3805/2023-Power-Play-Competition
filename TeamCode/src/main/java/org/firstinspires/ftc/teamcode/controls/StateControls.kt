package org.firstinspires.ftc.teamcode.controls

import com.atomicrobotics.cflib.Constants
import com.atomicrobotics.cflib.controls.Controls

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

    }
}