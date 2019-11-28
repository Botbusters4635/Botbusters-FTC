package org.firstinspires.ftc.teamcode.core

class Path(vararg val coordinates: Coordinate): Iterable<Coordinate> {

    val size: Int
        get() = coordinates.size

    operator fun get(index: Int): Coordinate {
        return coordinates[index]
    }

    override fun iterator(): Iterator<Coordinate> {
        return coordinates.iterator()
    }
}