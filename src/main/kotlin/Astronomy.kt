import net.e175.klaus.solarpositioning.AzimuthZenithAngle
import net.e175.klaus.solarpositioning.DeltaT
import net.e175.klaus.solarpositioning.SPA
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D
import java.util.*

object Astronomy {
    val xAxisOrientationToNorth = 246.0
    val latitude = 51.5111
    val longitude = -0.11714
    val elevation = 10.0
    val pressure = 1010.0
    val temperature = 19.0

    fun azimuthZenithToSpherical(azimuthZenith: AzimuthZenithAngle): Vector3D {
        return (Vector3D(1.0, Math.toRadians(azimuthZenith.zenithAngle), Math.toRadians(azimuthZenith.azimuth - xAxisOrientationToNorth)))
    }

    fun getSolarPosition(): Vector3D {
        val cal = GregorianCalendar()
        val position = SPA.calculateSolarPosition(cal, latitude, longitude, elevation, DeltaT.estimate(cal), pressure, temperature)
        // println("position is ${position.azimuth-246.0} ${position.zenithAngle}")
        return (azimuthZenithToSpherical(position))
    }

    fun getSolarRayCartesian(): Vector3D {
        return Geometry.sphericalToCartesian(getSolarPosition()).scalarMultiply(-1.0)
    }
}