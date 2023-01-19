package org.firstinspires.ftc.teamcode.utilities

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor


class Localiser(val robot: RobotConfig,val xSensor: ModernRoboticsI2cRangeSensor, val ySensor: ModernRoboticsI2cRangeSensor) {
    val currentPose: Pose2d
        get() {
            val x = xSensor.cmUltrasonic()
            val y = ySensor.cmUltrasonic()
            val heading = robot.botHeading.toDouble()

            val positionVector = Vector2d(x, y).rotated(-heading)

            return Pose2d(x, y, heading)
        }
}