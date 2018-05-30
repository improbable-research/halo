import com.google.gson.Gson
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D
import java.io.BufferedReader
import java.io.FileNotFoundException
import java.io.FileReader

class HelioStatCalibration : ArrayList<HelioStatCalibration.DataPoint> {
    class DataPoint(val ABC : Vector3D, val control: ServoSetting) {}

    constructor(array : List<DataPoint>) : super(array) {}
    constructor() : super() {}

    fun readFromFile(filename: String) {
        val gson = Gson()
        var buff = BufferedReader(FileReader("calibrationData.json"))

        val data = gson.fromJson(buff, CalibrationRawData::class.java)

        for(entry in data) {
            println(entry.key)
            println(entry.value.servoPositions)
            println(entry.value.data.plane.ABCD.split(",").map(String::toDouble))
            val abcd = entry.value.data.plane.ABCD.split(",").map(String::toDouble)
            val plane = Vector3D(abcd[0], abcd[1], abcd[2]).normalize().scalarMultiply(abcd[3])
            add(
                    DataPoint(
                            plane,
                            ServoSetting(entry.value.servoPositions["209"]?:0, entry.value.servoPositions["210"]?:0)
                    )
            )
        }

    }

}