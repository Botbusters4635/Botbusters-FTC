package org.firstinspires.ftc.teamcode.core

data class Coordinate(var x: Double = 0.0, var y: Double = 0.0){
    fun closeTo(targetCoord: Coordinate, nearness: Double = 10.0): Boolean {
        return this.x in (targetCoord.x - targetCoord.x * nearness / 100)..(targetCoord.x + targetCoord.x * nearness / 100) && this.y in (targetCoord.y - targetCoord.y * nearness / 100)..(targetCoord.y + targetCoord.y * nearness / 100)
    }
}