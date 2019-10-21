package ftcsim

import javafx.scene.paint.Color
import javafx.scene.layout.Region
import javafx.scene.shape.Rectangle
import kotlin.math.PI
import kotlin.math.atan2
import kotlin.math.pow
import kotlin.math.sqrt
import kotlin.text.*

class DriveTrainUpdate {
    var xSpeed = 0.0
    var ySpeed = 0.0
    var angularSpeed = 0.0
}

class DriveTrain (frameWidth : Double, frameHeight : Double, wheelWidth : Double, wheelRadius : Double) : Region() {
    private var frameWidth = 0.0
    private var frameHeight = 0.0
    private var wheelWidth = 0.0
    private var wheelRadius = 0.0

    var topRightSpeed = 1
    var topLeftSpeed = 0.0

    var bottomRightSpeed = 1
    var bottomLeftSpeed = 0.0

    init {
        this.frameWidth = frameWidth
        this.frameHeight =  frameHeight
        this.wheelWidth = wheelWidth
        this.wheelRadius = wheelRadius

        val frame = Rectangle(0.0, 0.0, frameWidth, frameHeight)
        frame.x = -frame.width / 2.0
        frame.y = -frame.height / 2.0
        frame.fill = null
        frame.stroke = Color.BLACK

        val topRightWheel = Rectangle(0.0, 0.0, wheelWidth, wheelRadius * 2)
        val topLeftWheel = Rectangle(0.0, 0.0, wheelWidth, wheelRadius * 2)
        val bottomRightWheel = Rectangle(0.0, 0.0, wheelWidth, wheelRadius * 2)
        val bottomLeftWheel = Rectangle(0.0, 0.0, wheelWidth, wheelRadius * 2)

        val orientationIndicator = Rectangle(0.0, 0.0, wheelRadius, wheelWidth)
        orientationIndicator.fill = null
        orientationIndicator.stroke = Color.BLUE

        orientationIndicator.x = -orientationIndicator.width/2
        orientationIndicator.y = -orientationIndicator.height/2 - frameHeight/2

        topRightWheel.x = frame.x - topRightWheel.width/2 + frameWidth
        topRightWheel.y = frame.y - topRightWheel.height/2

        topLeftWheel.x = frame.x - topLeftWheel.width / 2
        topLeftWheel.y = frame.y - topLeftWheel.height / 2

        bottomRightWheel.x =  frame.x - bottomRightWheel.width/2 + frameWidth
        bottomRightWheel.y =  frame.y - bottomRightWheel.height / 2 + frameHeight

        bottomLeftWheel.x =  frame.x - bottomLeftWheel.width / 2
        bottomLeftWheel.y =  frame.y - bottomLeftWheel.height / 2 + frameHeight

        this.children.add(frame)
        this.children.add(topRightWheel)
        this.children.add(topLeftWheel)
        this.children.add(bottomRightWheel)
        this.children.add(bottomLeftWheel)
        this.children.add(orientationIndicator)
    }

    fun update(timeStep: Double) : DriveTrainUpdate{


        /**
         * Modified to behave properly with Graphics Library Coordinate System
         *
         * This is in local reference, need to convert to world reference before applying to simulation
         */
        val state = DriveTrainUpdate()
        state.ySpeed = -(topLeftSpeed + topRightSpeed + bottomLeftSpeed + bottomRightSpeed) * (wheelRadius / 4) * timeStep
        state.xSpeed = -(-topLeftSpeed + topRightSpeed + bottomLeftSpeed - bottomRightSpeed) * (wheelRadius / 4) * timeStep
        state.angularSpeed = -(-topLeftSpeed + topRightSpeed - bottomLeftSpeed + bottomRightSpeed) * (wheelRadius / (4*(frameHeight / 2 + frameWidth / 2))) * timeStep
        state.angularSpeed = state.angularSpeed * 180.0 / PI

        return state
    }
}