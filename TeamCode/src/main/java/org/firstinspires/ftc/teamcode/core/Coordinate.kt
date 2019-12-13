package org.firstinspires.ftc.teamcode.core

import kotlin.math.absoluteValue

data class Coordinate(var x: Double = 0.0, var y: Double = 0.0){
    fun closeTo(targetCoord: Coordinate, threshold: Double = 0.01): Boolean {

        val deltaX = targetCoord.x - this.x
        val deltaY = targetCoord.y - this.y

        val xOnTarget = deltaX.absoluteValue < threshold
        val yOnTarget = deltaY.absoluteValue < threshold

        return xOnTarget && yOnTarget
    }
}