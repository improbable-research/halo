class StudentKt() {

    lateinit var name: String
    var age: Int = 0

    constructor(name: String, age: Int): this() {
        this.name = name
        this.age = age
    }

    override fun toString(): String {
        return "StudentKt [name = $name, age = $age]"
    }
}
