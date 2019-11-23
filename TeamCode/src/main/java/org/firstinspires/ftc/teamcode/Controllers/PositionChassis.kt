package org.firstinspires.ftc.teamcode.Controllers

import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.core.Coordinate
import org.firstinspires.ftc.teamcode.core.PID
import org.firstinspires.ftc.teamcode.core.PIDSettings
import org.firstinspires.ftc.teamcode.core.Twist2D

class PositionChassis : Chassis() {

    var targetCoords = Coordinate(0.0, 0.0)
        set(value){
            field = value
            xPID.target = targetCoords.x
            yPID.target = targetCoords.y
        }

    val xPID = PID(PIDSettings(1.0, 0.0, 0.0))
    val yPID = PID(PIDSettings(1.0, 0.0, 0.0))

    var timeReference = ElapsedTime()
    val maxVelocityChange = 1.0  //Per second

    var lastVx = 0.0
    var lastVY = 0.0


    override fun update() {
        super.update()

        xPID.target = targetCoords.x
        yPID.target = targetCoords.y

        telemetry.addData("xError", xPID.error)
        telemetry.addData("yError", yPID.error)

        movementTarget.vx = xPID.update(currentCoords.x)
        movementTarget.vy = yPID.update(currentCoords.y)
    }
}