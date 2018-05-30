class HelioStatCalibration : ArrayList<HelioStatCalibration.DataPoint> {
    class DataPoint(val ABC : List<Double>, val control: List<Int>) {}

    constructor(array : List<DataPoint>) : super(array) {}
    constructor() : super() {}
}