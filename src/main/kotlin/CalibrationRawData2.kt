class CalibrationRawData2 {
    val captures = ArrayList<DataPoint>()

    class DataPoint {
        val ABCD = ArrayList<Double>(4)
        var A1 : Int = 0
        var A2 : Int = 0
    }
}