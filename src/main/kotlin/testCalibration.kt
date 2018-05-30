
fun main(args : Array<String>) {
    val calibrator = HelioStatCalibration()
    calibrator.readFromFile("calibrationData.json")
    calibrator.inferMaxAPosteriori()
}