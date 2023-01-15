package org.firstinspires.ftc.teamcode.teleop

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.teleop.TeleopVariables.slidesDown
import org.firstinspires.ftc.teamcode.teleop.TeleopVariables.slidesLow
import org.firstinspires.ftc.teamcode.teleop.TeleopVariables.slidesMid
import org.firstinspires.ftc.teamcode.teleop.TeleopVariables.slidesHigh
import org.firstinspires.ftc.teamcode.teleop.TeleopVariables.clawOpenPos
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
    var clawOpenPos=0.62
    @JvmField
    var clawClosePos=0.8
    @JvmField
    var slidesLow=0
    @JvmField
    var slidesMid=600
    @JvmField
    var slidesHigh=1000
    @JvmField
    var slidesDown=0
}
@TeleOp(name="Turret Teleop", group="TeleOp")
class OurTeleOp : LinearOpMode() {
    companion object {
        const val max: Double = 1.0
        const val some: Double = 0.75
        const val half: Double = 0.5
        const val why: Double = 0.25
    }
    private var robotConfig: RobotConfig?=null
    private var vars: VariableStuff?=null

    private enum class LiftState {
        MANUAL, DOWN, LOW, MID, HIGH
    }

    override fun runOpMode() {
        val dashboard=FtcDashboard.getInstance()
        var telemetry=MultipleTelemetry(telemetry, dashboard.telemetry)
        telemetry.addLine("Robot has been turned on. Run for your life!")
        telemetry.update()
        robotConfig=RobotConfig(hardwareMap)
        vars= VariableStuff

        /*** controller settings  */
        val lightRumble = 0.4
        val strongRumble = 0.75
        var lastLeftBump=false
        var lastRightBump=false
        var leftBump=false
        var rightBump=false
        var lastliftState=false
        /*** A few more variables  */
        var m1=.5 //Speed multiplier
        telemetry.addLine("Robot is ready. Run for your life!")
        telemetry.update()
        while(!opModeIsActive()) { //loop after Init, before Start
            rumble1("both",1.0, 100) //run vibrator to tell drivers that they need to start the opmode
            rumble2("both",1.0, 100)
        }

        robotConfig!!.slides.mode=DcMotor.RunMode.STOP_AND_RESET_ENCODER
        robotConfig!!.slides.mode=DcMotor.RunMode.RUN_USING_ENCODER
        var liftState=LiftState.MANUAL
        var slideHeight=0
        waitForStart()


        /*** add timers for everything that needs them */
        var lastTime=System.currentTimeMillis()

        while(opModeIsActive()) {
            /*** loop frame time system */
            val currentSystemTime=System.currentTimeMillis()
            telemetry.addData("Time between frame: ", currentSystemTime-lastTime)
            lastTime=currentSystemTime
            rightBump=gamepad2.right_bumper
            leftBump=gamepad2.left_bumper
            var slideResetState=robotConfig!!.slidesReset.state

            /**** slides control state machine */
            when(liftState){
                LiftState.MANUAL -> if(gamepad2.right_bumper){
                    liftState= LiftState.DOWN
                }
                else if(!lastliftState){ //check if reset switch is triggered (need to check if it was not triggered last time)
                    robotConfig!!.slides.mode=DcMotor.RunMode.STOP_AND_RESET_ENCODER
                }
                else{
                    robotConfig!!.slides.mode=DcMotor.RunMode.RUN_WITHOUT_ENCODER
                    robotConfig!!.slides.power=gamepad2.left_stick_y.toDouble()
                }
                LiftState.DOWN -> if(gamepad2.ps){
                    liftState= LiftState.MANUAL
                }
                else if(rightBump && !lastRightBump){
                    if(slideHeight == slidesDown) {
                        slideHeight = slidesLow
                        robotConfig!! slidesGo low withPower -some
                        //robotConfig!!.clawAngle.position=TeleopVariables.clawDownLow
                        liftState= LiftState.LOW
                    }
                    else when(slideHeight){
                        slidesMid -> liftState= LiftState.MID
                        slidesLow -> liftState= LiftState.LOW
                        slidesHigh -> liftState= LiftState.HIGH
                    }
                }
                else if(leftBump && !lastLeftBump){
                    if(slideHeight== slidesDown) {
                        gamepad2.rumble(1.0, 0.0, 250)
                    }
                    else when(slideHeight){
                        slidesMid -> liftState= LiftState.MID
                        slidesLow -> liftState= LiftState.LOW
                        slidesHigh -> liftState= LiftState.HIGH
                    }
                }
                LiftState.LOW -> if(gamepad2.ps){
                    liftState= LiftState.MANUAL
                }
                else if(rightBump && !lastRightBump){
                    if(slideHeight== slidesLow) {
                        slideHeight = slidesMid
                        robotConfig!! slidesGo mid withPower -some
                        //robotConfig!!.clawAngle.position=TeleopVariables.clawDownMid
                        liftState= LiftState.MID
                    }
                    else when(slideHeight){
                        slidesMid -> liftState= LiftState.MID
                        slidesDown -> liftState= LiftState.DOWN
                        slidesHigh -> liftState= LiftState.HIGH
                    }
                }
                else if(leftBump && !lastLeftBump){
                    if(slideHeight== slidesLow) {
                        slideHeight = slidesDown
                        robotConfig!! slidesGo down withPower some
                        liftState= LiftState.DOWN
                    }
                    else when(slideHeight){
                        slidesMid -> liftState= LiftState.MID
                        slidesDown -> liftState= LiftState.DOWN
                        slidesHigh -> liftState= LiftState.HIGH
                    }
                }
                LiftState.MID -> if(gamepad2.ps){
                    liftState= LiftState.MANUAL
                }
                else if(rightBump && !lastRightBump){
                    if(slideHeight== slidesMid) {
                        slideHeight = slidesHigh
                        robotConfig!! slidesGo high withPower -some
                        //robotConfig!!.clawAngle.position=TeleopVariables.clawDownHigh
                        liftState= LiftState.HIGH
                    }
                    else when(slideHeight){
                        slidesLow -> liftState= LiftState.LOW
                        slidesDown -> liftState= LiftState.DOWN
                        slidesHigh -> liftState= LiftState.HIGH
                    }
                }
                else if(leftBump && !lastLeftBump){
                    if(slideHeight== slidesMid) {
                        slideHeight = slidesLow
                        robotConfig!! slidesGo low withPower some
                        //robotConfig!!.clawAngle.position=TeleopVariables.clawDownLow
                        liftState= LiftState.LOW
                    }
                    else{
                        when(slideHeight){
                            slidesLow -> liftState= LiftState.LOW
                            slidesDown -> liftState= LiftState.DOWN
                            slidesHigh -> liftState= LiftState.HIGH
                        }
                    }
                }
                LiftState.HIGH -> if(gamepad2.ps){
                    liftState= LiftState.MANUAL
                }
                else if(rightBump && !lastRightBump){
                    if(slideHeight== slidesHigh) {
                        gamepad2.rumble(0.0, 1.0, 250)
                        robotConfig!! slidesGo high withPower -some
                    }
                    else when(slideHeight){
                        slidesMid -> liftState= LiftState.MID
                        slidesDown -> liftState= LiftState.DOWN
                        slidesLow -> liftState= LiftState.LOW
                    }
                }
                else if (leftBump && !lastLeftBump){
                    if(slideHeight == slidesHigh) {
                        slideHeight = slidesMid
                        robotConfig!! slidesGo mid withPower some
                        //robotConfig!!.clawAngle.position=TeleopVariables.clawDownMid
                        liftState= LiftState.MID
                    }
                    else when(slideHeight){
                        slidesMid -> liftState= LiftState.MID
                        slidesDown -> liftState= LiftState.DOWN
                        slidesLow -> liftState= LiftState.LOW
                    }
                }
            }
            lastliftState=slideResetState
            lastLeftBump=leftBump
            lastRightBump=rightBump

            if(abs(gamepad2.right_stick_y)<.1) {
                robotConfig!!.slides.mode=DcMotor.RunMode.RUN_TO_POSITION
                robotConfig!!.slides.targetPosition=robotConfig!!.slides.currentPosition
                robotConfig!!.slides.power= why
            }
            else {
                robotConfig!!.slides.mode=DcMotor.RunMode.RUN_USING_ENCODER
                robotConfig!!.slides.power = -gamepad2.right_stick_y.toDouble()
            }

            /***** slides control ****/
            when {
                gamepad2.square ->{
                    robotConfig!!.claw.position= clawOpenPos
                }
                gamepad2.circle ->{
                    robotConfig!!.claw.position= TeleopVariables.clawClosePos
                }
                /*gamepad2.dpad_up ->{
                    robotConfig!!.clawAngle.position=TeleopVariables.clawUp
                }
                gamepad2.dpad_down ->{
                    robotConfig!!.clawAngle.position=TeleopVariables.clawDown
                }
                gamepad2.dpad_left ->{
                    robotConfig!!.clawAngle.position=TeleopVariables.clawDown-.15
                }
                gamepad2.dpad_right ->{
                    robotConfig!!.clawAngle.position=TeleopVariables.clawDown-.25
                }*/
            }
            val dpadup = gamepad2.dpad_up
            val dpaddown = gamepad2.dpad_down
            var dpadupnum: Double = if (dpadup){
                1.0
            } else{
                0.0
            }
            var dpaddownnum: Double = if (dpaddown){
                1.0
            } else{
                0.0
            }
            //robotConfig!!.slides.power=gamepad2.left_stick_y.toDouble()
            //robotConfig!!.slides.power=-gamepad2.right_stick_y.toDouble()

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

            /******************* Warning, math ahead  */
            var drive=gamepad1.left_stick_x.toDouble()
            var strafe=-gamepad1.right_stick_x.toDouble()
            var rotate=-gamepad1.left_stick_y.toDouble()
            robotConfig!!.bl.power=m1*(drive+rotate+strafe)
            robotConfig!!.br.power=m1*(drive-rotate+strafe)
            robotConfig!!.fr.power=m1*(-drive+rotate+strafe)
            robotConfig!!.fl.power=m1*(-drive-rotate+strafe)

            /**************** Telemetry Stuff *****************/
            /* telemetry.addData("ry", gamepad1.right_stick_y); //use when motor connections/commands messed up
            telemetry.addData("rx", gamepad1.right_stick_x);
            telemetry.addData("lx", gamepad1.left_stick_x);
            telemetry.addData("fl", robotConfig!!.fl.power);
            telemetry.addData("bl", robotConfig!!.bl.power);
            telemetry.addData("fr", robotConfig!!.fr.power);
            telemetry.addData("br", robotConfig!!.br.power);
            telemetry.addData("br", robotConfig.br.getCurrentPosition());
            telemetry.addData("fr", robotConfig.fr.getCurrentPosition()); */
            //telemetry.addLine("Random Stuff \n")
            telemetry.addData("slides position", robotConfig!!.slides.currentPosition)
            telemetry.addData("slides State", liftState)
            telemetry.update()

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