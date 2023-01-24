package org.firstinspires.ftc.teamcode.teleop

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.outoftheboxrobotics.photoncore.PhotonCore
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.teleop.TeleopVariables.clawClosePos
import org.firstinspires.ftc.teamcode.teleop.TeleopVariables.slidesDown
import org.firstinspires.ftc.teamcode.teleop.TeleopVariables.slidesLow
import org.firstinspires.ftc.teamcode.teleop.TeleopVariables.slidesMid
import org.firstinspires.ftc.teamcode.teleop.TeleopVariables.slidesHigh
import org.firstinspires.ftc.teamcode.teleop.TeleopVariables.clawOpenPos
import org.firstinspires.ftc.teamcode.teleop.TeleopVariables.parallel
import org.firstinspires.ftc.teamcode.teleop.TeleopVariables.perpendicular
import org.firstinspires.ftc.teamcode.utilities.CursedCode.Companion.aboveGround
import org.firstinspires.ftc.teamcode.utilities.CursedCode.Companion.down
import org.firstinspires.ftc.teamcode.utilities.CursedCode.Companion.high
import org.firstinspires.ftc.teamcode.utilities.CursedCode.Companion.low
import org.firstinspires.ftc.teamcode.utilities.CursedCode.Companion.mid
import org.firstinspires.ftc.teamcode.utilities.RobotConfig
import org.firstinspires.ftc.teamcode.utilities.VariableStuff
import kotlin.math.abs


/**
 **************************************This code has been modified by FTC team 12051 NotNotNerds*************************************
 **********************We do not guarantee that your robot will function correctly after you have used this code*********************
 ********************************It is not recommended that you use our modifications with your robot********************************
 */
@Config()
object TeleopVariables {
    @JvmField
    var clawOpenPos=0.52
    @JvmField
    var clawClosePos=0.6
    @JvmField
    var slidesLow=1400
    @JvmField
    var slidesMid=2600
    @JvmField
    var slidesHigh=3650
    @JvmField
    var slidesDown=0
    @JvmField
    var cone5 = 400 //1400 remove
    @JvmField
    var cone4 = 300 //1200 remove
    @JvmField
    var cone3 = 200 //1000 remove
    @JvmField
    var cone2 = 100 //800 remove
    @JvmField
    var cone1 = 0 //300 remove
    @JvmField
    var slidesAboveGround = 300
    @JvmField
    var slidePower = -1.0
    @JvmField
    var parallel = 0.85
    @JvmField
    var perpendicular = 0.52
}
@TeleOp(name="Working Teleop", group="TeleOp")
class OurTeleOp : LinearOpMode() {
    companion object {
        const val max: Double = 1.0
        const val some: Double = 0.75
        const val half: Double = 0.5
        const val why: Double = 0.25
    }
    private var vars: VariableStuff?=null

    private enum class LiftState {
        MANUAL, AUTO
    }

    override fun runOpMode() {
        PhotonCore.enable()
        val dashboard=FtcDashboard.getInstance()
        var telemetry=MultipleTelemetry(telemetry, dashboard.telemetry)
        telemetry.addLine("Robot has been turned on. Run for your life!")
        telemetry.update()
        val robotConfig=RobotConfig(hardwareMap)
        vars= VariableStuff

        /*** controller settings  */
        val lightRumble = 0.4
        val strongRumble = 0.75
        var lastliftState=false
        /*** A few more variables  */
        var m1=.5 //Speed multiplier
        telemetry.addLine("Robot is ready. Run for your life!")
        telemetry.update()
        while(!opModeIsActive()) { //loop after Init, before Start
            rumble1("both",1.0, 100) //run vibrator to tell drivers that they need to start the opmode
            rumble2("both",1.0, 100)
        }

        robotConfig.slides.mode=DcMotor.RunMode.STOP_AND_RESET_ENCODER
        robotConfig.slides.mode=DcMotor.RunMode.RUN_USING_ENCODER
        var liftState=LiftState.MANUAL
        var slideHeight=0
        var coneGrabbed = false
        waitForStart()


        /*** add timers for everything that needs them */
        var lastTime=System.currentTimeMillis()

        while(opModeIsActive()) {
            /*** loop frame time system */
            val currentSystemTime=System.currentTimeMillis()
            telemetry.addData("Time between frame: ", currentSystemTime-lastTime)
            lastTime=currentSystemTime

            /*** bulk read stuff ***/
            var slideResetState=robotConfig.slidesReset.state
            val slidesEncoderPos=robotConfig.slides.currentPosition
            if (slidesEncoderPos < 750){
                val distanceToCone = robotConfig.cone.getDistance(DistanceUnit.INCH)
                telemetry.addData("Distance to cone", distanceToCone)
                /**** Auto-gripping control ***/
                if (distanceToCone < 2.3 && slidesEncoderPos < 500) {
                    robotConfig.claw.position = clawClosePos
                    coneGrabbed = true
                } else {
                    coneGrabbed = false
                }
            }

            /**** new slides fsm ***/
            when(liftState) {
                LiftState.MANUAL -> if (gamepad2.circle || gamepad2.square || gamepad2.triangle || gamepad2.cross || coneGrabbed) {
                    liftState = LiftState.AUTO
                } else if (!lastliftState && slideResetState) { //check if reset switch is triggered (need to check if it was not triggered last time)
                    robotConfig.slides.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
                } else if (abs(slidesEncoderPos) > 5300 ) {
                    robotConfig.slides.power = gamepad2.left_trigger.toDouble()
                }
                else {
                    robotConfig.slides.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
                    robotConfig.slides.power =
                        gamepad2.right_trigger.toDouble() - gamepad2.left_trigger.toDouble()
                }
                LiftState.AUTO -> if (gamepad2.cross){
                    slideHeight = slidesDown
                    robotConfig slidesGo down withPower some
                }
                else if (gamepad2.triangle){
                    slideHeight = slidesHigh
                    robotConfig slidesGo high withPower some
                }
                else if (gamepad2.square){
                    slideHeight = slidesMid
                    robotConfig slidesGo mid withPower some
                }
                else if (gamepad2.circle){
                    slideHeight = slidesLow
                    robotConfig slidesGo low withPower some
                }
                else if (coneGrabbed){
                    robotConfig slidesGo aboveGround withPower max
                }
                else if (gamepad2.ps || gamepad2.right_trigger.toDouble() > 0.25 || gamepad2.left_trigger.toDouble() > 0.25)
                {
                    liftState = LiftState.MANUAL
                }
            }

                lastliftState=slideResetState


            /***** claw control ****/
            when {
                gamepad2.right_bumper ->{
                    robotConfig.claw.position= clawClosePos
                    coneGrabbed = false
                }
                gamepad2.left_bumper ->{
                    robotConfig.claw.position= clawOpenPos
                    coneGrabbed = false
                }
            }

            /***************** drive stuff beneath here  */
            when {
                gamepad1.circle -> { //drive speed multiplier modifiers
                    m1=.4
                    rumble1("r", lightRumble, 100)
                }
                gamepad1.cross -> {
                    m1=.2
                    rumble1("r", lightRumble, 100)
                }
                gamepad1.triangle -> {
                    m1=1.0
                    rumble1("r", lightRumble, 100)
                }
                gamepad1.square -> {
                    m1=.7
                    rumble1("r", lightRumble, 100)
                }
            }

            /******************* Warning, math ahead  */ //I'm lazy and therefore will NOT be using the rcdrive/gamepaddrive functions
            var drive=-gamepad1.left_stick_y.toDouble()
            var strafe=-gamepad1.right_stick_x.toDouble()
            var rotate=-gamepad1.left_stick_x.toDouble()
            robotConfig.bl.power=m1*(drive+rotate-strafe)
            robotConfig.br.power=m1*(drive-rotate+strafe)
            robotConfig.fr.power=m1*(drive+rotate+strafe)
            robotConfig.fl.power=m1*(drive-rotate-strafe)

            /**************** Telemetry Stuff *****************/
            /* telemetry.addData("ry", gamepad1.right_stick_y); //use when motor connections/commands messed up
            telemetry.addData("rx", gamepad1.right_stick_x);
            telemetry.addData("lx", gamepad1.left_stick_x);
            telemetry.addData("fl", robotConfig.fl.power);
            telemetry.addData("bl", robotConfig.bl.power);
            telemetry.addData("fr", robotConfig.fr.power);
            telemetry.addData("br", robotConfig.br.power);
            telemetry.addData("br", robotConfig.br.getCurrentPosition());
            telemetry.addData("fr", robotConfig.fr.getCurrentPosition()); */
            //telemetry.addLine("Random Stuff \n")
            telemetry.addData("slides position", slidesEncoderPos)
            telemetry.addData("Slides target", slideHeight)
            telemetry.addData("slides State", liftState)
            telemetry.update()


            robotConfig.parallelEncoderServo.position = parallel
            robotConfig.perpendicularEncoderServo.position = perpendicular

        }
    }
    /*** functions used to simplify long lines of code into slightly shorter ones */
    private fun rumble1(side: String, rumblePower : Double, duration : Int){
        when {
            side.lowercase() == "l" -> {
                gamepad1.rumble(rumblePower, 0.0, duration)
            }
            side.lowercase() == "r" -> {
                gamepad1.rumble(0.0, rumblePower, duration)
            }
            side.lowercase() == "both" -> {
                gamepad1.rumble(rumblePower/2, rumblePower/2, duration)
            }
            else -> {
                telemetry.addLine("One of the rumble commands has been set up incorrectly, make sure the side is only one letter, l or r")
            }
        }
    }

    private fun rumble2(side: String,rumblePower: Double, duration : Int){
        when {
            side.lowercase() == "l" -> {
                gamepad2.rumble(rumblePower, 0.0, duration)
            }
            side.lowercase() == "r" -> {
                gamepad2.rumble(0.0, rumblePower, duration)
            }
            side.lowercase() == "both" -> {
                gamepad2.rumble(rumblePower/2, rumblePower/2, duration)
            }
            else -> {
                telemetry.addLine("One of the rumble commands has been set up incorrectly, make sure the side is only one letter, l or r")
            }
        }
    }


}