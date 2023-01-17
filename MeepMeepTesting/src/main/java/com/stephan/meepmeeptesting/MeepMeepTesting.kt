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
                    15.0) // Option: Set theme. Default = ColorSchemeRedDark()
                .setColorScheme(ColorSchemeRedDark()).followTrajectorySequence { drive ->
                    drive.trajectorySequenceBuilder(Pose2d(0.0, 0.0, 0.0)).forward(30.0)
                        .turn(Math.toRadians(90.0)).forward(30.0).addDisplacementMarker {}
                        .turn(Math.toRadians(90.0)).splineTo(Vector2d(10.0, 15.0), 0.0)
                        .turn(Math.toRadians(90.0)).build()
                }

        // Set field image
        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_KAI_DARK)
            .setDarkMode(true) // Background opacity from 0-1
            .setBackgroundAlpha(0.95f).addEntity(myBot).start()
    }
}