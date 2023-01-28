package org.firstinspires.ftc.teamcode.drive

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.util.Encoder

/*
* Sample tracking wheel localizer implementation assuming the standard configuration:
*
*    ^
*    |
*    | ( x direction)
*    |
*    v
*    <----( y direction )---->

*        (forward)
*    /--------------\
*    |     ____     |
*    |     ----     |    <- Perpendicular Wheel
*    |           || |
*    |           || |    <- Parallel Wheel
*    |              |
*    |              |
*    \--------------/
*
*/
class TwoWheelTrackingLocalizer(hardwareMap: HardwareMap, private val drive: SampleMecanumDrive) :
    TwoTrackingWheelLocalizer(listOf(Pose2d(PARALLEL_X, PARALLEL_Y, 0.0),
        Pose2d(PERPENDICULAR_X, PERPENDICULAR_Y, Math.toRadians(90.0)))) {
    // Parallel/Perpendicular to the forward axis
    // Parallel wheel is parallel to the forward axis
    // Perpendicular is perpendicular to the forward axis
    private val parallelEncoder: Encoder
    private val perpendicularEncoder: Encoder
    override fun getHeading(): Double {
        return drive.rawExternalHeading
    }

    override fun getHeadingVelocity(): Double? {
        return drive.getExternalHeadingVelocity()
    }
    var X_MULTIPLIER = (102.5/71.30)*(102.5/103.0) // Multiplier in the X direction
    var Y_MULTIPLIER = (106.0/71.0) // Multiplier in the Y direction

    override fun getWheelPositions(): List<Double> {
        return listOf(encoderTicksToInches(parallelEncoder.currentPosition.toDouble()*X_MULTIPLIER),
            encoderTicksToInches(perpendicularEncoder.currentPosition.toDouble())*Y_MULTIPLIER)
    }

    override fun getWheelVelocities(): List<Double> {
        return listOf(encoderTicksToInches(parallelEncoder.correctedVelocity*X_MULTIPLIER),
            encoderTicksToInches(perpendicularEncoder.correctedVelocity*Y_MULTIPLIER))
    }

    companion object {
        var TICKS_PER_REV = 8192.0
        var WHEEL_RADIUS = 0.68897638 // in
        var GEAR_RATIO = 1.0 // output (wheel) speed / input (encoder) speed
        var PARALLEL_X = 0.0 // X is the up and down direction
        var PARALLEL_Y = 2.5 // Y is the strafe direction
        var PERPENDICULAR_X = -10.0
        var PERPENDICULAR_Y = 0.0
        fun encoderTicksToInches(ticks: Double): Double {
            return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV
        }
    }

    init {
        parallelEncoder = Encoder(hardwareMap.get(DcMotorEx::class.java, "parallel"))
        perpendicularEncoder = Encoder(hardwareMap.get(DcMotorEx::class.java, "perpendicular"))

        //reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
        perpendicularEncoder.direction = Encoder.Direction.REVERSE
    }
}