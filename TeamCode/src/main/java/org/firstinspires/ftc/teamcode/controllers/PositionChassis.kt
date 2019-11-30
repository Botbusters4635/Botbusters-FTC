package org.firstinspires.ftc.teamcode.controllers

import android.os.SystemClock
import kotlinx.coroutines.isActive
import kotlinx.coroutines.runBlocking
import org.firstinspires.ftc.teamcode.core.Coordinate
import org.firstinspires.ftc.teamcode.core.PID
import org.firstinspires.ftc.teamcode.core.PIDSettings
import org.firstinspires.ftc.teamcode.core.Path
import kotlin.math.absoluteValue
import kotlin.math.pow
import kotlin.math.sqrt

class PositionChassis : Chassis() {

    var targetCoords = Coordinate(0.0, 0.0)

    val xPID = PID(PIDSettings(2.5, 0.0, 0.0))
    val yPID = PID(PIDSettings(2.5, 0.0, 0.0))

    val maxVelocityChange = 0.5

    var currentVx = 0.0
    var currentVy = 0.0

    var maxV = 0.5

    var followingPath = false

    var runInverse = false

    val distanceToTarget
        get() = sqrt((targetCoords.x - currentCoords.x).pow(2.0) + Math.pow(targetCoords.y - currentCoords.y, 2.0))

    val onTarget
        get() = distanceToTarget < 0.05

    override fun update() {
        super.update()

        if(!followingPath) return

        if(onTarget){
            movementTarget.vx = 0.0
            movementTarget.vy = 0.0
            movementTarget.theta = heading
            return
        }

        xPID.target = targetCoords.x
        yPID.target = targetCoords.y
        var targetVx = xPID.update(currentCoords.x)
        var targetVy = yPID.update(currentCoords.y)

        targetVx = targetVx.coerceIn(-maxV, maxV)
        targetVy = targetVy.coerceIn(-maxV, maxV)


        if(Math.abs(targetVx - currentVx) * timeStep > maxVelocityChange * timeStep){
            currentVx += Math.copySign(maxVelocityChange * timeStep, targetVx)
        }else{
            currentVx = targetVx
        }


        if(Math.abs(targetVy - currentVy) * timeStep > maxVelocityChange * timeStep){
            currentVy += Math.copySign(maxVelocityChange * timeStep, targetVy)
        }else{
            currentVy = targetVy
        }

        val headingInRadians = degreesToRadians(heading)

        movementTarget.vx = currentVx * Math.cos(headingInRadians) + currentVy * Math.sin(headingInRadians)
        movementTarget.vy = currentVy * Math.cos(headingInRadians) - currentVx * Math.sin(headingInRadians)
    }

    fun runToPosition(target: Coordinate) = runBlocking{
        targetCoords = target
        followingPath = true
        while(!onTarget && isActive){

        }
        followingPath = false
    }

    fun turnToAngle(targetAngle: Double) = runBlocking {
        movementTarget.theta = targetAngle
        val startTime = SystemClock.elapsedRealtime() / 1000.0
        var currentTime = 0.0
        while((heading - targetAngle).absoluteValue > 1 && isActive){
            currentTime = SystemClock.elapsedRealtime() / 1000.0 - startTime

            if(currentTime > 2.0){
                break
            }
        }
    }

    fun followPath(path: Path) = runBlocking {
        for(coordinate in path){
            runToPosition(coordinate)

        }
    }

}