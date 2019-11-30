package org.firstinspires.ftc.teamcode.core

class Path(vararg val waypoints: Waypoint): Iterable<Waypoint> {

    val size: Int
        get() = waypoints.size

    operator fun get(index: Int): Waypoint {
        return waypoints[index]
    }

    override fun iterator(): Iterator<Waypoint> {
        return waypoints.iterator()
    }
}