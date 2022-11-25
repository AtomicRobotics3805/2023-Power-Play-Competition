@file:Suppress("PropertyName")

package com.atomicrobotics.cflib.driving.localizers

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.atomicrobotics.cflib.Constants.drive
import com.atomicrobotics.cflib.Constants.opMode
import com.atomicrobotics.cflib.trajectories.addRotated
import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.util.ElapsedTime
import kotlin.math.*

/**
 * Untested
 */
class DistanceSensorLocalizer(val constants: DistanceSensorLocalizerConstants) : SubsystemLocalizer {

    @Suppress("UNUSED_PARAMETER")
    override var poseEstimate: Pose2d
        get() = _poseEstimate
        set(value) {}
    override val poseVelocity: Pose2d?
        get() = _poseVelocity

    private var _poseEstimate = Pose2d()
    private var _poseVelocity: Pose2d? = null

    private var timer: ElapsedTime? = null

    private lateinit var parallelDistanceSensor: AnalogInput
    private lateinit var perpendicularDistanceSensor: AnalogInput

    override fun initialize() {
        parallelDistanceSensor = opMode.hardwareMap.get(AnalogInput::class.java, constants.PARALLEL_NAME)
        perpendicularDistanceSensor = opMode.hardwareMap.get(AnalogInput::class.java, constants.PERPENDICULAR_NAME)
    }

    override fun update() {
        val parallelDistance = constants.voltageToInches(parallelDistanceSensor.voltage) *
                if (constants.REVERSE_PARALLEL_SENSOR) -1 else 1
        val perpendicularDistance = constants.voltageToInches(perpendicularDistanceSensor.voltage) *
                if (constants.REVERSE_PERPENDICULAR_SENSOR) -1 else 1
        val canSeeHorizontalWall = canSeeHorizontalWall()
        if (canSeeHorizontalWall.first xor canSeeHorizontalWall.second) {
            val xPosition = (if (canSeeHorizontalWall.first)
                parallelDistance * sin(drive.rawExternalHeading) else
                perpendicularDistance * cos(drive.rawExternalHeading)) +
                -constants.HALF_FIELD_SIZE * sign(drive.rawExternalHeading) * sign(parallelDistance)
            val yPosition = (if (canSeeHorizontalWall.second)
                parallelDistance * sin(drive.rawExternalHeading) else
                perpendicularDistance * cos(drive.rawExternalHeading)) +
                    constants.HALF_FIELD_SIZE * -sign(drive.rawExternalHeading) * sign(perpendicularDistance)
            val newPose = Pose2d(xPosition, yPosition, drive.rawExternalHeading)
            if (timer != null) {
                _poseVelocity = (newPose - _poseEstimate) / timer!!.seconds()
                _poseEstimate = newPose
                timer!!.reset()
            }
            else {
                timer = ElapsedTime()
            }
        }
        else {
            _poseVelocity = null
        }
    }

    /**
     * If both sensors are looking at the same wall, then the robot has no way of knowing what its
     * other coordinate is. This function essentially creates lines pointing in the direction that
     * the sensors are looking, and then determines where those lines intersect the horizontal field
     * walls. It then returns a boolean for whether the parallel distance sensor sees the horizontal
     * field wall and whether the perpendicular distance sensor sees the horizontal field wall
     */
    private fun canSeeHorizontalWall(): Pair<Boolean, Boolean> {
        val parallelPosition = drive.poseEstimate.addRotated(constants.PARALLEL_POSITION)
        val perpendicularPosition = drive.poseEstimate.addRotated(constants.PERPENDICULAR_POSITION)

        val parallelHorizontalWallDistance = -parallelPosition.y * sign(parallelPosition.heading) *
                (if (constants.REVERSE_PARALLEL_SENSOR) -1 else 1) + constants.HALF_FIELD_SIZE
        val perpendicularHorizontalWallDistance = -perpendicularPosition.y * sign(perpendicularPosition.heading) *
                (if (constants.REVERSE_PERPENDICULAR_SENSOR) -1 else 1) + constants.HALF_FIELD_SIZE

        val parallelHorizontalIntercept = (parallelHorizontalWallDistance /
                tan(parallelPosition.heading) * sign(parallelPosition.heading)) + parallelPosition.x
        val perpendicularHorizontalIntercept = perpendicularHorizontalWallDistance /
                tan(perpendicularPosition.heading + PI / 2) * sign(perpendicularPosition.heading) + perpendicularPosition.x

        val parallelSeesHorizontal = (abs(parallelHorizontalIntercept) < constants.HALF_FIELD_SIZE)
        val perpendicularSeesHorizontal = (abs(perpendicularHorizontalIntercept) < constants.HALF_FIELD_SIZE)

        return Pair(parallelSeesHorizontal, perpendicularSeesHorizontal)
    }
}

interface DistanceSensorLocalizerConstants {

    val HALF_FIELD_SIZE: Double
    val PARALLEL_NAME: String
    val PERPENDICULAR_NAME: String
    val REVERSE_PARALLEL_SENSOR: Boolean
    val REVERSE_PERPENDICULAR_SENSOR: Boolean
    val PARALLEL_POSITION: Vector2d
    val PERPENDICULAR_POSITION: Vector2d

    fun voltageToInches(voltage: Double): Double
}