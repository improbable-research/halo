import com.google.gson.Gson
import java.io.BufferedReader
import java.io.FileReader

fun main(args : Array<String>) {
    val gson = Gson()
    val buff = BufferedReader(FileReader("calibrationData.json"))

    val data = gson.fromJson(buff, CalibrationRawData::class.java)

    for(entry in data) {
        println(entry.key)
        println(entry.value.servoPositions)
        println(entry.value.data.plane.ABCD.split(",").map(String::toDouble))

    }
}