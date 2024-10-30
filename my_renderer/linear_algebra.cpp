#include <array>
#include <cassert>
#include "linear_algebra.h"

Vector2::Vector2() {};
Vector2::Vector2(std::initializer_list<float> list) {
	assert(list.size() == 2);

	x = *list.begin();
	y = *(list.begin() + 1);
}

Vector3::Vector3() {};
Vector3::Vector3(std::initializer_list<float> list) {
	assert(list.size() == 3);

	x = *list.begin();
	y = *(list.begin() + 1);
	z = *(list.begin() + 2);
}

Vector4 Vector3::extend(float w) {
	return { x, y, z, w };
}

Vector4::Vector4() {};
Vector4::Vector4(std::initializer_list<float> list) {
	assert(list.size() == 4);

	x = *list.begin();
	y = *(list.begin() + 1);
	z = *(list.begin() + 2);
	w = *(list.begin() + 3);
}

Matrix4::Matrix4() {};
Matrix4::Matrix4(std::initializer_list<float> list) {
	assert(list.size() == 16);

	for (size_t i = 0; i < 16; i++) {
		data[i / 4][i % 4] = *(list.begin() + i);
	}
}

Vector4 operator*(Matrix4 matrix, Vector4 vector) {
	Vector4 result;

	for (size_t i = 0; i < 4; i++) {
		float row_sum = 0;
		for (size_t j = 0; j < 4; j++) {
			row_sum += matrix[i][j] * vector[j];
		}

		result[i] = row_sum;
	}

	return result;
}