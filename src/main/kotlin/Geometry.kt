import net.e175.klaus.solarpositioning.AzimuthZenithAngle
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D

object Geometry {


    fun cartesianToSpherical(xyz : Vector3D) : Vector3D {
        val standardSpherical = Vector3D(xyz.norm, Math.acos(xyz.y/xyz.norm), Math.atan2(xyz.z,xyz.x))
        if(standardSpherical.z > 3.0*Math.PI/4.0 || standardSpherical.z < -Math.PI/4.0) {
            return erectToFlacid(standardSpherical)
        }
        return standardSpherical
    }


    fun standardCartesianToSpherical(xyz : Vector3D) : Vector3D {
        return Vector3D(xyz.norm, Math.acos(xyz.y/xyz.norm), Math.atan2(xyz.z,xyz.x))
    }

    fun erectToFlacid(spherical : Vector3D) : Vector3D {
        val rotationPerturbation : Double = if(spherical.z > 0.0) {
            -Math.PI
        } else {
            Math.PI
        }
        return Vector3D(spherical.x,
                        -spherical.y,
                        spherical.z + rotationPerturbation )
    }

    fun sphericalToCartesian(spherical : Vector3D) : Vector3D {
        return Vector3D(
                spherical.x * Math.sin(spherical.y) * Math.cos(spherical.z),
                spherical.x * Math.cos(spherical.y),
                spherical.x * Math.sin(spherical.y) * Math.sin(spherical.z)
        )
    }
}