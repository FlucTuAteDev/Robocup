template<typename T>
T clamp(T value, T min, T max) {
	return value < min ? min : (value > max ? max : value);
}

template<typename T>
T sign(T num) {
	return (num > 0) - (num < 0);
}