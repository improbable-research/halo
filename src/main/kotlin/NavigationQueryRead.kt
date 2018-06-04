import com.google.gson.Gson
import java.io.BufferedReader
import java.io.FileReader

class NavigationQueryRead(val query: HelioStatNavigator.NavigationQuery) {

    fun readFromFile(filename: String) {
        val gson = Gson()
        val buff = BufferedReader(FileReader(filename))
        val data = gson.fromJson(buff, NavigationRawQuery::class.java)

        for(entry in data.entries) {
            // TODO
            // Remember to convert params to probabilistic params
            val params = entry.value.params.params.split(",").map(String::toDouble)
            val pivotPoint = params[0]
            val mPitch = params[1]
            val cPitch = params[2]
            val mRotation = params[3]
            val cRotation = params[4]
            val servoSetting = entry.value.presentControl.servoSettings.values
//            val currentServoSetting = ServoSetting(servoSetting.first())
        }
    }
}