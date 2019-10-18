class Matrix2D private constructor(private val matrixValues: MutableList<MutableList<Double>>) {
    var rows: Int = 0
    var columns: Int = 0
    init {
        columns = matrixValues[0].size
        rows = matrixValues.size
    }
    operator fun get(i: Int, j: Int): Double{
        return matrixValues[i][j]
    }
    operator fun set(i: Int, j: Int, value: Double){
        matrixValues[i][j] = value
    }
    operator fun times(matrix2: Matrix2D): Matrix2D{
        assert(columns == matrix2.rows)
        val newMatrix = Matrix2D.zeroMatrixOf(rows, matrix2.columns)
        for(i in 0 until rows){
            for(j in 0 until matrix2.columns){
                newMatrix[j, i] = 0.0
                 for(k in 0 until columns){
                    newMatrix[i, j] += matrix2.[k, j] * matrixValues[]
                }
            }
        }
    }

    companion object {
        fun zeroMatrixOf(columns: Int, rows: Int): Matrix2D{
            val row = mutableListOf<Double>()
            for(i in 0 until columns){
                row.add(0.0)
            }
            val matrixValues = mutableListOf<MutableList<Double>>()
            for(i in 0 until rows){
                matrixValues.add(row)
            }
            return Matrix2D(matrixValues)
        }
        fun matrixOf(matrix: Array<Array<Double>>): Matrix2D{
            val columnNumber = matrix[0].size
            val matrixValues = mutableListOf<MutableList<Double>>()
            for(row in matrix){
                if(row.size != columnNumber){
                    throw Exception("Number of columns is not equal across rows!")
                }
                matrixValues.add(row.toMutableList())
            }
            return Matrix2D(matrixValues)
        }
    }
}