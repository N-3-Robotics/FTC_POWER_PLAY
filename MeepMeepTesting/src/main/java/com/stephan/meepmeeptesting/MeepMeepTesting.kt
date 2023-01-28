import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.noahbres.meepmeep.MeepMeep
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity


object MeepMeepTesting {
    @JvmStatic
    fun main(args: Array<String>) {
        // Declare a MeepMeep instance
        // With a field size of 800 pixels
        val meepMeep = MeepMeep(1000)
        val myBot: RoadRunnerBotEntity =
            DefaultBotBuilder(meepMeep) // Required: Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(45.0,
                    30.0,
                    Math.toRadians(315.0),
                    Math.toRadians(60.0),
                    12.4) // Option: Set theme. Default = ColorSchemeRedDark()
                .setColorScheme(ColorSchemeRedDark()).followTrajectorySequence { drive ->
                    drive.trajectorySequenceBuilder(Pose2d(37.0, -66.0, Math.toRadians(90.0)))
                        .splineTo(Vector2d(30.0, -27.0), Math.toRadians(135.0))
                        .UNSTABLE_addTemporalMarkerOffset(0.0){
/*
                            drive.claw.position=ClawOpen
*/
                        }
                        .lineTo(Vector2d(36.5, -31.5))
                        .lineToLinearHeading(Pose2d(40.0, -12.0, Math.toRadians(0.0)))
                        .UNSTABLE_addTemporalMarkerOffset(0.0){
/*                            drive.slides.targetPosition=cone5
                            drive.slides.mode= DcMotor.RunMode.RUN_TO_POSITION
                            drive.slides.power=slidePower*/
                        }
                        .lineTo(Vector2d(66.0, -12.0))
 .UNSTABLE_addTemporalMarkerOffset(0.0){
/*
     drive.claw.position=ClawClose
*/
 }
 .UNSTABLE_addTemporalMarkerOffset(0.5){
/*     drive.slides.targetPosition= slidesLow+200
     drive.slides.mode= DcMotor.RunMode.RUN_TO_POSITION
     drive.slides.power=slidePower*/
 }
                        .lineTo(Vector2d(59.0, -12.0))
                        .lineToLinearHeading(Pose2d(53.0, -20.0, Math.toRadians(-135.0)))
                        .UNSTABLE_addDisplacementMarkerOffset(0.0){ //open claw
/*
                            drive.claw.position = ClawOpen
*/
                        }
                        .UNSTABLE_addTemporalMarkerOffset(0.5){ //move slides to cone4 level after bit of time, to prevent collision
/*                            drive.slides.targetPosition= cone4
                            drive.slides.mode= DcMotor.RunMode.RUN_TO_POSITION
                            drive.slides.power=slidePower*/
                        }
                        .lineTo(Vector2d(59.0, -18.0))
                        .lineToLinearHeading(Pose2d(59.0, -12.0, Math.toRadians(0.0)))

/*** loop 2 ***/
                        .lineTo(Vector2d(69.0, -13.0)) //go to cone stack
                        .UNSTABLE_addDisplacementMarkerOffset(0.0){ //close claw
/*
                            drive.claw.position = ClawClose
*/
                        }
                        .UNSTABLE_addTemporalMarkerOffset(0.2){ //raise slides
/*                            drive.slides.targetPosition= slidesLow+200
                            drive.slides.mode= DcMotor.RunMode.RUN_TO_POSITION
                            drive.slides.power=slidePower*/
                        }
                        .waitSeconds(.2) //wait a bit for slides to go up to low height
                        .lineTo(Vector2d(59.0, -14.5)) //back up a bit from the stack
                        .lineToLinearHeading(Pose2d(53.0, -20.0, Math.toRadians(-135.0))) //go to pole while rotating
                        .UNSTABLE_addDisplacementMarkerOffset(2.5){ //release cone
/*
                            drive.claw.position = ClawOpen
*/
                        }
                        .forward(1.0) //strafe left a bit from pole
                        .strafeLeft(2.5)
                        .back(3.0)
                        .turn(Math.toRadians(-45.0))
                        .strafeRight(7.5)
                        .forward(45.0)
 .build()
     }

// Set field image
meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
 .setDarkMode(false) // Background opacity from 0-1
 .setBackgroundAlpha(0.95f).addEntity(myBot).start()
}
}