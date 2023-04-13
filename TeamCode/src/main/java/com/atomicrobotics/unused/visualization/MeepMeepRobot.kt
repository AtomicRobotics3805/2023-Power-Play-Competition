package com.atomicrobotics.unused.visualization

import com.atomicrobotics.unused.CommandGroup
import com.atomicrobotics.unused.Constants
import com.atomicrobotics.unused.driving.drivers.Driver

data class MeepMeepRobot(val driver: Driver, val width: Double, val length: Double,
                         val routine: () -> CommandGroup, val color: Constants.Color)