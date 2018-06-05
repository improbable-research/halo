import com.google.gson.Gson

class Json {

    companion object {

        private val gson = Gson()

        fun <T> toJson(obj: T): String {
            return gson.toJson(obj)
        }

        fun <T> fromJson(json: String, type: Class<T>): T {
            return gson.fromJson(json, type)
        }
    }

}
