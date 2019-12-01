package org.firstinspires.ftc.teamcode.core

class Path(vararg val waypoints: Coordinate): Iterable<Coordinate> {

    val size: Int
        get() = waypoints.size

    operator fun get(index: Int): Coordinate {
        return waypoints[index]
    }

    override fun iterator(): Iterator<Coordinate> {
        return waypoints.iterator()
    }
}