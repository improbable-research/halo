package httpRequests

import com.fasterxml.jackson.databind.ObjectMapper
import com.google.gson.Gson

class JsonUtils {

    companion object {

//        private val mapper = ObjectMapper()
//
//        fun <T> toJson(obj: T): String {
//            return mapper.writerWithDefaultPrettyPrinter().writeValueAsString(obj)
//        }
//
//        fun <T> jsonToClass(json: String, type: Class<T>): T {
//            return mapper.readValue(json, type)
//        }

        private val gson = Gson()

        fun <T> toJson(obj: T): String {
            return gson.toJson(obj)
        }

        fun <T> fromJson(json: String, type: Class<T>): T {
            return gson.fromJson(json, type)
        }
    }

}
