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
        val meepMeep = MeepMeep(800)
        val myBot: RoadRunnerBotEntity =
            DefaultBotBuilder(meepMeep) // Required: Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60.0,
                    60.0,
                    Math.toRadians(180.0),
                    Math.toRadians(180.0),
                    12.0) // Option: Set theme. Default = ColorSchemeRedDark()
                .setColorScheme(ColorSchemeRedDark()).followTrajectorySequence { drive ->
                    drive.trajectorySequenceBuilder(Pose2d(-36.0, -66.0, Math.toRadians(90.0))) //-35, -65, 90
                        .lineToLinearHeading(Pose2d(-69.0, -14.0, Math.toRadians(180.0))) //go to stack
                        .UNSTABLE_addDisplacementMarkerOffset(0.0){ //wait for claw to close
                        }.UNSTABLE_addTemporalMarkerOffset(0.2){ //raise slides
                        }
                        .waitSeconds(0.2) //wait for slides to go up a bit
                        .lineTo(Vector2d(-57.0, -12.0)) //back up a bit from the stack
                        .lineToLinearHeading(Pose2d(-56.0, -25.0, Math.toRadians(-45.0))) //go to pole while rotating
                        .UNSTABLE_addDisplacementMarkerOffset(0.0){ //open claw
                        }

/*                        .lineTo(Vector2d(-50.0, -42.0))
                        .lineToLinearHeading(Pose2d(-64.0, -12.0, Math.toRadians(180.0)))
                        .lineToLinearHeading(Pose2d(-52.0, -20.0, Math.toRadians(-45.0)))
                        .lineToLinearHeading(Pose2d(-64.0, -12.0, Math.toRadians(180.0)))
                        .lineToLinearHeading(Pose2d(-52.0, -20.0, Math.toRadians(-45.0)))
                        .lineToLinearHeading(Pose2d(-64.0, -12.0, Math.toRadians(180.0)))
                        .lineToLinearHeading(Pose2d(-52.0, -20.0, Math.toRadians(-45.0)))
                        .lineToLinearHeading(Pose2d(-64.0, -12.0, Math.toRadians(180.0)))
                        .lineToLinearHeading(Pose2d(-52.0, -20.0, Math.toRadians(-45.0)))
                        .lineToLinearHeading(Pose2d(-64.0, -12.0, Math.toRadians(180.0)))*/
                        //.lineToLinearHeading(Pose2d(-36.00, -36.00, Math.toRadians(0.0))) // at tile (1, 0)
                       // .addSpatialMarker(Vector2d(-36.00, -36.00)) {
                     /*       drive.SLIDES.targetPosition = midPole
                            drive.SLIDES.mode = DcMotor.RunMode.RUN_TO_POSITION
                            drive.SLIDES.power = 1.0
                            while (drive.SLIDES.isBusy) {
                                telemetry.addData("slides", drive.SLIDES.currentPosition)
                                telemetry.update()
                            }
                            drive.SLIDES.power = 0.0
                            drive.SLIDES.mode = DcMotor.RunMode.RUN_USING_ENCODER*/
                        //}
                        //.lineToLinearHeading(Pose2d(-26.50, -26.50, Math.toRadians(45.00))) // at junction (1, 1)
                        /*.addSpatialMarker(Vector2d(-26.50, -26.50)) {
                         *//*   openClaw()
                            sleep(500)
                            closeClaw()*//*
                        }
                        .lineToLinearHeading(Pose2d(-36.00, -36.00, Math.toRadians(90.00))) // at tile (1, 1)
                        .lineToLinearHeading(Pose2d(-36.00, -12.00, Math.toRadians(90.00)))// at tile (1, 2)
                        .lineToLinearHeading(Pose2d(-65.00, -12.00, Math.toRadians(180.00)))// // at cone stack
                        .lineToLinearHeading(Pose2d(-36.00, -12.00, Math.toRadians(90.00))) // at tile (1, 2)
                        .lineToLinearHeading(Pose2d(-27.00, -3.50, Math.toRadians(45.00)))// at junction (1, 2)
                        .lineToLinearHeading(Pose2d(-36.00, -36.00, Math.toRadians(90.00))) // at tile (1, 1)
                        .lineToLinearHeading(Pose2d(-60.00, -36.00, Math.toRadians(180.00))) // at tile (0, 1), parked*/
                        .build()
                }

        // Set field image
        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_KAI_DARK)
            .setDarkMode(true) // Background opacity from 0-1
            .setBackgroundAlpha(0.95f).addEntity(myBot).start()
    }
}