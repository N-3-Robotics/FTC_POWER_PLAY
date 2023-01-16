package org.firstinspires.ftc.teamcode.utilities

import org.firstinspires.ftc.teamcode.teleop.TeleopVariables
import org.firstinspires.ftc.teamcode.utilities.DriveConstants.aboveGround

/** I don't recommend ever looking into this file, especially since it keeps the code working well
 * And please don't question my decision of adding this file in the first place
 * Thinking about this will keep you awake for hours */

class CursedCode {
    companion object{
        const val down="down"
        const val low="low"
        const val high="high"
        const val mid="mid"
        const val aboveGround="aboveGround"
    }
    infix fun go(position:String): Int{
        return when(position){
            high -> TeleopVariables.slidesHigh
            mid -> TeleopVariables.slidesMid
            low -> TeleopVariables.slidesLow
            down -> TeleopVariables.slidesDown
            aboveGround -> TeleopVariables.slidesAboveGround
            else -> 0
        }
    }

}