package org.firstinspires.ftc.teamcode.controllers

import android.os.SystemClock
import kotlinx.coroutines.delay
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

    val maxVelocityChange = 0.5

    var currentVx = 0.0

    var maxVx = 0.5

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
            movementTarget.theta = getHeading()
            return
        }

        xPID.target = 0.0
        var targetVx = xPID.update(-distanceToTarget)

        targetVx = targetVx.coerceIn(0.0, maxVx)

        if(Math.abs(targetVx - currentVx) * timeStep > maxVelocityChange * timeStep){
            currentVx += Math.copySign(maxVelocityChange * timeStep, targetVx)
        }else{
            currentVx = targetVx
        }


        val angle = Math.atan2(targetCoords.y - currentCoords.y, targetCoords.x - currentCoords.x) * 180.0 / Math.PI
        var targetHeading = angle + if(runInverse) 180 else 0

        telemetry.addData("angular.js", angle)
        telemetry.addData("no u", targetHeading)
        telemetry.update()

        if(targetHeading > 180.0){
            targetHeading -= 360.0
        }

        if(targetHeading < -180.0){
            targetHeading += 360.0
        }


        movementTarget.vx = currentVx * if(runInverse) -1 else 1
        movementTarget.vy = 0.0
        movementTarget.theta = targetHeading
    }

    fun runToPosition(target: Coordinate, runInverse: Boolean = false) = runBlocking{
        targetCoords = target
        followingPath = true
        this@PositionChassis.runInverse = runInverse
        while(!onTarget && isActive){

        }
        followingPath = false
    }

    fun turnToAngle(targetAngle: Double) = runBlocking {
        movementTarget.theta = targetAngle
        val startTime = SystemClock.elapsedRealtime() / 1000.0
        var currentTime = 0.0
        while((getHeading() - targetAngle).absoluteValue > 1 && isActive){
            currentTime = SystemClock.elapsedRealtime() / 1000.0 - startTime

            if(currentTime > 2.0){
                break
            }
        }
    }

    fun followPath(path: Path, runInverse: Boolean = false) = runBlocking {
        for(coordinate in path){
            runToPosition(coordinate, runInverse)

        }
    }

}
