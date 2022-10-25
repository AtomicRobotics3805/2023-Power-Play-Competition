package com.atomicrobotics.cflib.visualization

import com.atomicrobotics.cflib.CommandGroup
import com.atomicrobotics.cflib.Constants
import com.atomicrobotics.cflib.driving.drivers.Driver

data class MeepMeepRobot(val driver: Driver, val width: Double, val length: Double,
                         val routine: () -> CommandGroup, val color: Constants.Color)