import com.google.gson.Gson
import io.improbable.keanu.vertices.dbl.probabilistic.GaussianVertex
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D
import java.io.BufferedReader
import java.io.FileReader

class NavigationQueryRead(val query: HelioStatNavigator.NavigationQuery): ArrayList<HelioStatNavigator.NavigationQuery> () {

    fun readFromFile(filename: String){
        val gson = Gson()
        val buff = BufferedReader(FileReader(filename))
        val data = gson.fromJson(buff, NavigationRawQuery::class.java)

        for(entry in data.entries) {

            val params = entry.value.params.params.split(",").map(String::toDouble)
            val pivotPoint = ProbabilisticVector3D(GaussianVertex(params[0], 0.1),
                                                   GaussianVertex(params[1], 0.1),
                                                   GaussianVertex(params[2], 0.1))
            val mPitch = params[3]
            val cPitch = params[4]
            val mRotation = params[5]
            val cRotation = params[6]
            val probabilisticParams = ProbabilisticHelioStatParameters(pivotPoint,
                    ProbabilisticHelioStatParameters.ServoParameters(GaussianVertex(mPitch, 0.1),
                                                                     GaussianVertex(cPitch, 0.1)),
                    ProbabilisticHelioStatParameters.ServoParameters(GaussianVertex(mRotation, 0.1),
                                                                     GaussianVertex(cRotation, 0.1)))

            val servoSetting = entry.value.presentControl.servoSettings.values
            val currentServoSetting = ServoSetting(servoSetting.first(), servoSetting.last())

            val sourceGeometry = entry.value.geometry.source.split(",").map(String::toDouble)
            val source = Vector3D(sourceGeometry[0], sourceGeometry[1], sourceGeometry[2])

            val targetGeometry = entry.value.geometry.targetPoint.split(",").map(String::toDouble)
            val targetPoint = Vector3D(targetGeometry[0], targetGeometry[1], targetGeometry[2])

            val query = HelioStatNavigator.NavigationQuery(probabilisticParams, currentServoSetting, source, targetPoint)

            this.add(query)
        }
    }
}