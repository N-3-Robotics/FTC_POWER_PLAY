package org.firstinspires.ftc.teamcode.robot

import com.acmerobotics.robomatic.util.PIDController
import com.qualcomm.robotcore.hardware.*
import org.firstinspires.ftc.teamcode.utilities.DriveConstants.HoldingPower
import org.firstinspires.ftc.teamcode.utilities.DriveConstants.SlidesTolerance
import kotlin.math.abs

class Slides(private val motor: DcMotorEx) {

    var state = State.STOPPED

    var correction: Double = 0.0
    var error: Double = 0.0

    var slidesPIDController = PIDController(0.5, 0.0, 0.0)

    enum class State{
        MOVING, STOPPED
    }

    var mode: DcMotor.RunMode
        get() = motor.mode
        set(value) {
            motor.mode = value
        }

    var power: Double
        get() = motor.power
        set(value) {
            motor.power = value
        }

    var zeroPowerBehavior: DcMotor.ZeroPowerBehavior
        get() = motor.zeroPowerBehavior
        set(value) {
            motor.zeroPowerBehavior = value
        }

    val currentPosition: Double
        get() = motor.currentPosition.toDouble()

    val currentVelocity: Double
        get() = motor.velocity.toDouble()

    var targetPosition = 0

    fun stop(){
        motor.power = HoldingPower
        state = State.STOPPED
    }

    fun goTo(position: Int) {
        state = State.MOVING
        motor.targetPosition = position
        motor.mode = DcMotor.RunMode.RUN_TO_POSITION
        motor.power = 0.5

    }

    fun update(){
        when (state) {
            State.MOVING -> {
                while (abs(error) > SlidesTolerance) {
                    error = targetPosition - currentPosition
                    correction = slidesPIDController.update(error)
                    power = correction
                }
                stop()
            }
            State.STOPPED -> {
                motor.power = 0.0
            }
        }
    }

    init {
        slidesPIDController.setOutputBounds(-1.0, 1.0)
    }


}