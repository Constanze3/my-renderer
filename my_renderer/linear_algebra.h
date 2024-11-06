#pragma once
#include <array>

typedef struct Vector4 Vector4;
typedef struct Vector3 Vector3;
typedef struct Vector2 Vector2;
typedef struct Matrix4 Matrix4;

struct Vector4 {
	float x = 0;
	float y = 0;
	float z = 0;
	float w = 0;

	Vector4();
	Vector4(std::initializer_list<float> list);

	inline float& operator[](size_t index) {
		switch (index) {
		case 0:
			return this->x;
		case 1:
			return this->y;
		case 2:
			return this->z;
		case 3:
			return this->w;
		default:
			__assume(false);
		}
	}
};

struct Vector3 {
	float x = 0;
	float y = 0;
	float z = 0;

	Vector3();
	Vector3(std::initializer_list<float> list);
	Vector4 extend(float w) const;
	Vector2 xy() const;
};

struct Vector2 {
	float x = 0;
	float y = 0;

	Vector2();
	Vector2(std::initializer_list<float> list);
};

struct Matrix4 {
	std::array<std::array<float, 4>, 4> data = {0};

	Matrix4();
	Matrix4(std::initializer_list<float> list);

	inline std::array<float, 4> operator[](size_t index) {
		return data[index];
	}
};

Vector4 operator*(Matrix4 matrix, Vector4 vector);