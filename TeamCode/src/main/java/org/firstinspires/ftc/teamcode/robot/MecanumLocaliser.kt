package org.firstinspires.ftc.teamcode.robot

import org.firstinspires.ftc.teamcode.robot.kinematics.Kinematics
import org.firstinspires.ftc.teamcode.robot.kinematics.MecanumKinematics
import org.firstinspires.ftc.teamcode.utilities.geometry.Pose2d
import org.firstinspires.ftc.teamcode.utilities.math.Angle

class MecanumLocaliser(private val robot: Robot, private val useExternalHeading: Boolean = true): Localiser {
    private var _poseEstimate = Pose2d()
    override var poseEstimate: Pose2d
        get() = _poseEstimate
        set(value) {
            lastWheelPositions = emptyList()
            lastExtHeading = Double.NaN
            if (useExternalHeading) robot.globalAngle = value.heading
            _poseEstimate = value
        }
    override var poseVelocity: Pose2d? = null
        private set
    private var lastWheelPositions = emptyList<Double>()
    private var lastExtHeading = Double.NaN

    override fun update() {
        val wheelPositions = robot.getWheelPositions()
        val extHeading = if (useExternalHeading) robot.globalAngle else Double.NaN
        if (lastWheelPositions.isNotEmpty()){
            val wheelDeltas = wheelPositions.zip(lastWheelPositions).map { it.first - it.second }

            val robotPoseDelta = MecanumKinematics.wheelToRobotVelocities(wheelDeltas, robot.trackWidth, robot.wheelBase, robot.lateralMultiplier)

            val finalHeadingDelta = if (useExternalHeading) Angle.normDelta(extHeading - lastExtHeading) else robotPoseDelta.heading

            _poseEstimate = Kinematics.relativeOdometryUpdate(_poseEstimate, Pose2d(robotPoseDelta.vec(), finalHeadingDelta))
        }

        val wheelVelocities = robot.getWheelVelocities()
        val extHeadingVel = robot.getExternalHeadingVelocity()

        if (wheelVelocities != null){
            poseVelocity = MecanumKinematics.wheelToRobotVelocities(wheelVelocities, robot.trackWidth, robot.wheelBase, robot.lateralMultiplier)
            if (useExternalHeading && extHeadingVel != null) {
                poseVelocity = Pose2d(poseVelocity!!.vec(), extHeadingVel)
            }
        }

        lastWheelPositions = wheelPositions
        lastExtHeading = extHeading
    }
}