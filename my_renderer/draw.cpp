#include <iostream>
#include <cmath>
#include <unordered_set>
#include <vector>
#include <cassert>
#include <array>
#include <functional>
#include <unordered_map>
#include <typeindex>
#include <any>
#include <limits>
#include <exception>

#include "linear_algebra.h"

float interpolate(float a, float b, float c, Vector3 distribution) {
	return distribution.x * a + distribution.y * b + distribution.z * c;
}

template<typename T>
T clamp(T min, T max, T value) {
	return std::max(min, std::min(value, max));
}

struct Color {
	float red;
	float green;
	float blue;

	Color(float red, float green, float blue) : 
		red(red), green(green), blue(blue) {}

	bool is_valid() {
		bool valid = true;

		valid &= 0 <= red && red <= 1;
		valid &= 0 <= green && green <= 1;
		valid &= 0 <= blue && blue <= 1;

		return valid;
	}
};

Color operator+(Color a, Color b) {
	float red = a.red + b.red;
	float green = a.green + b.green;
	float blue = a.blue + b.blue;

	return Color(red, green, blue);
}

Color operator*(Color color, float multiplier) {
	float red = color.red * multiplier;
	float green = color.green * multiplier;
	float blue = color.blue * multiplier;

	return Color(red, green, blue);
}

Color operator*(float multiplier, Color color) {
	return color * multiplier;
}

Color operator/(Color color, float divisor) {
	float red = color.red / divisor;
	float green = color.green / divisor;
	float blue = color.blue / divisor;

	return Color(red, green, blue);
}

struct Varying {
	std::any value = std::any();
	std::function<Varying(Varying, Varying)> op_addition = std::function<Varying(Varying, Varying)>();
	std::function<Varying(Varying, float)> op_multiplication_1 = std::function<Varying(Varying, float)>();
	std::function<Varying(float, Varying)> op_multiplication_2 = std::function<Varying(float, Varying)>();
	std::function<Varying(Varying, float)> op_division = std::function<Varying(Varying, float)>();

	Varying() {}

	template<typename T>
	Varying(T value) :
		value(value),
		op_addition([](Varying a, Varying b) -> Varying{
			return Varying(a.cast<T>() + b.cast<T>());
		}),
		op_multiplication_1([](Varying v, float multiplier) -> Varying {
			return Varying(v.cast<T>() * multiplier);
		}),
		op_multiplication_2([](float multiplier, Varying v) -> Varying {
			return Varying(multiplier * v.cast<T>());
		}),
		op_division([](Varying v, float divisor) -> Varying {
			return Varying(v.cast<T>() / divisor);
		})
	{}

	// copy
	//Varying(const Varying& other) : 
	//	value(other.value),
	//	op_addition(other.op_addition),
	//	op_multiplication_1(other.op_multiplication_1),
	//	op_multiplication_2(other.op_multiplication_2),
	//	op_division(other.op_division) 
	//{};

	//// move
	//Varying(Varying&& other) noexcept : 
	//	value(other.value),
	//	op_addition(other.op_addition),
	//	op_multiplication_1(other.op_multiplication_1),
	//	op_multiplication_2(other.op_multiplication_2),
	//	op_division(other.op_division)
	//{}

	template<typename T>
	Varying& operator=(const T& other)
	{
		*this = Varying(other);
		return *this;
	}

	template<typename T>
	T cast() const {
		return std::any_cast<T>(value);
	}

	//template<typename T>
	//Varying& operator=(T&& other) { 
	//	*this = Varying(other);
	//	return *this;
	//}
};

Varying operator+(const Varying a, const Varying b) {
	return a.op_addition(a, b);
}

Varying operator*(const Varying value, float multiplier) {
	return value.op_multiplication_1(value, multiplier);
}

Varying operator*(float multiplier, const Varying value) {
	return value.op_multiplication_2(multiplier, value);
}

Varying operator/(const Varying value, float divisor) {
	return value.op_division(value, divisor);
}

Varying interpolate(const Varying a, const Varying b, const Varying c, const Vector3 distribution) {
	return distribution.x * a + distribution.y * b + distribution.z * c;
}

using StringVaryingMap = std::unordered_map<std::string, Varying>;

struct TriangleIndices {
	size_t v1 = 0;
	size_t v2 = 0;
	size_t v3 = 0;

	TriangleIndices(std::initializer_list<size_t> list) {
		assert(list.size() == 3);

		v1 = *list.begin();
		v2 = *(list.begin() + 1);
		v3 = *(list.begin() + 2);
	}
};

// A buffer of BGR values.
// The size of the buffer is 3 * width * height.
struct Canvas {
	size_t width;
	size_t height;
	uint8_t* buffer;

	Canvas(uint8_t* buffer, size_t width, size_t height) : buffer(buffer), width(width), height(height) {}

	void set_pixel_color(size_t x, size_t y, Color color) const {
		assert(buffer);

		//if (!color.is_valid()) {
		//	std::cout << color.red << " " << color.green << " " << color.blue << "\n";
		//	throw std::logic_error("Color intended to be painted on canvas is invalid.");
		//}

		size_t index = ((height - 1 - y) * width + x) * 3;
		uint8_t* pixel = buffer + index;

		uint8_t blue = static_cast<uint8_t>(std::roundf(color.blue * 255));
		uint8_t green = static_cast<uint8_t>(std::roundf(color.green * 255));
		uint8_t red = static_cast<uint8_t>(std::roundf(color.red * 255));

		pixel[0] = blue;
		pixel[1] = green;
		pixel[2] = red;
	}

	void clear(Color color) const {
		for (size_t x = 0; x < width; x++) {
			for (size_t y = 0; y < height; y++) {
				set_pixel_color(x, y, color);
			}
		}
	}
};

void draw_line(Canvas canvas, int startX, int startY, int endX, int endY, Color color) {
	size_t width = canvas.width;
	size_t height = canvas.height;

	bool inverted = false;

	if (abs(endX - startX) < abs(endY - startY)) {
		std::swap(startX, startY);
		std::swap(endX, endY);
		std::swap(width, height);
		inverted = true;
	}

	if (endX < startX) {
		std::swap(startX, endX);
		std::swap(startY, endY);
	}

	int dx = endX - startX;
	int dy = endY - startY;

	int yi = 1;

	if (dy < 0) {
		dy = -dy;
		yi = -1;
	}

	int d = 2 * dy - dx;
	int y = startY;

	for (size_t x = startX; x <= endX && 0 <= x && x < width; x++) {
		if (0 <= y && y < height) {
			if (!inverted) {
				canvas.set_pixel_color(x, y, color);
			}
			else {
				canvas.set_pixel_color(y, x, color);
			}
		}

		if (d > 0) {
			d -= 2 * dx;
			y += yi;
		}
		d += 2 * dy;
	}
}

struct EdgeHasher {
	inline size_t operator()(const std::pair<size_t, size_t>& p) const {
		std::hash<size_t> hasher;
		return hasher(p.first + p.second);
	}
};

std::unordered_set<std::pair<size_t, size_t>, EdgeHasher> triangles_to_edges(const std::vector<TriangleIndices>& triangles) {
	std::unordered_set<std::pair<size_t, size_t>, EdgeHasher> edges;

	for (const TriangleIndices& triangle : triangles) {
		size_t v1 = triangle.v1;
		size_t v2 = triangle.v2;
		size_t v3 = triangle.v3;

		edges.insert({ v1, v2 });
		edges.insert({ v2, v3 });
		edges.insert({ v3, v1 });
	}

	return edges;
}

bool is_top_or_left(Vector2 a, Vector2 b) {
	Vector2 ab = b - a;

	bool is_top = ab.y == 0 && ab.x > 0;
	bool is_left = ab.y > 0;

	return is_top || is_left;
}

// Magnitude of the cossproduct of vectors ab and ac.
float edge_function(Vector2 a, Vector2 b, Vector2 c) {
	return (c.x - b.x) * (a.y - b.y) - (a.x - b.x) * (c.y - b.y);
}

// Determines whether a point is inside a triangle.
// If yes then it also writes the barycentric coordinates of the point in terms of v1, v2, v3 to "barycentric".
bool is_in_triangle(Vector2 p, Vector2 v1, Vector2 v2, Vector2 v3, Vector3& barycentric, bool counter_clockwise) {
	float area = edge_function(v1, v2, v3);
	float w1 = edge_function(v2, v3, p);
	float w2 = edge_function(v3, v1, p);
	float w3 = edge_function(v1, v2, p);

	if (!counter_clockwise) {
		area = -area;
		w1 = -w1;
		w2 = -w2;
		w3 = -w3;
	}

	bool overlaps = true;
	overlaps &= w1 == 0 ? is_top_or_left(v2, v3) : w1 > 0;
	overlaps &= w2 == 0 ? is_top_or_left(v3, v1) : w2 > 0;
	overlaps &= w3 == 0 ? is_top_or_left(v1, v2) : w3 > 0;

	if (overlaps) {
		// the point overlaps triangle

		w1 /= area;
		w2 /= area;
		w3 /= area;

		barycentric = Vector3{ w1, w2, w3 };
		return true;
	}

	return false;
}

template<typename U>
void rasterize(
	Canvas canvas,
	const std::vector<Vector3>& vertices,
	const std::vector<TriangleIndices>& indices,
	std::vector<std::vector<float>>& depth_buffer,
	U uniforms,
	std::vector<StringVaryingMap>& varying,
	std::function<Color(U, const StringVaryingMap&)> fragment_shader
) {
	assert(0 < canvas.width);
	assert(0 < canvas.height);

	std::vector<float> ooz;
	// divide all varying values with z for perspective correct interpolation
	for (size_t i = 0; i < vertices.size(); i++) {
		for (auto& kv : varying[i]) {
			Varying& value = kv.second;
			value = value / vertices[i].z;
		}

		ooz.push_back(1 / vertices[i].z);
	}

	for (const TriangleIndices& triangle : indices) {
		Vector3 v1 = vertices[triangle.v1];
		Vector3 v2 = vertices[triangle.v2];
		Vector3 v3 = vertices[triangle.v3];

		float bbmin_x_float = std::floor(std::min({ v1.x, v2.x, v3.x }));
		float bbmin_y_float = std::floor(std::min({ v1.y, v2.y, v3.y }));
		float bbmax_x_float = std::floor(std::max({ v1.x, v2.x, v3.x }));
		float bbmax_y_float = std::floor(std::max({ v1.y, v2.y, v3.y }));

		int bbmin_x = static_cast<int>(bbmin_x_float);
		int bbmin_y = static_cast<int>(bbmin_y_float);
		int bbmax_x = static_cast<int>(bbmax_x_float);
		int bbmax_y = static_cast<int>(bbmax_y_float);

		bbmin_x = clamp(0, static_cast<int>(canvas.width) - 1, bbmin_x);
		bbmin_y = clamp(0, static_cast<int>(canvas.height) - 1, bbmin_y);
		bbmax_x = clamp(0, static_cast<int>(canvas.width) - 1, bbmax_x);
		bbmax_y = clamp(0, static_cast<int>(canvas.height) - 1, bbmax_y);

		for (size_t x = bbmin_x; x <= bbmax_x; x++) {
			for (size_t y = bbmin_y; y <= bbmax_y; y++) {
				float x_float = static_cast<float>(x);
				float y_float = static_cast<float>(y);

				Vector3 barycentric;

				if (is_in_triangle({ x_float, y_float }, v1.xy(), v2.xy(), v3.xy(), barycentric, true)) {
					float interpolated_z = 1 / interpolate(
						ooz[triangle.v1], 
						ooz[triangle.v2], 
						ooz[triangle.v3], 
						barycentric
					);

					if (depth_buffer[x][y] < interpolated_z) {
						// there is something in front of this fragment already
						continue;
					}

					// this fragment is in front
					depth_buffer[x][y] = interpolated_z;

					// transform varying values
					StringVaryingMap interpolated_varying;

					StringVaryingMap varying_v1 = varying[triangle.v1];
					StringVaryingMap varying_v2 = varying[triangle.v2];
					StringVaryingMap varying_v3 = varying[triangle.v3];

					// iterate and combine varying values
					for (auto const& kv : varying_v1) {
						std::string key = kv.first;

						Varying value_v1 = kv.second;
						Varying value_v2 = varying_v2.at(key);
						Varying value_v3 = varying_v3.at(key);

						Color a_color = value_v1.cast<Color>();
						Color b_color = value_v2.cast<Color>();
						Color c_color = value_v3.cast<Color>();

						// std::cout << barycentric.x + barycentric.y + barycentric.z << "\n";

						// multiply with interpolated_z for perspective correct interpolation 
						// (the division with z-values was done above before the rasterization loop)
						Varying value = interpolated_z * interpolate(value_v1, value_v2, value_v3, barycentric);
						interpolated_varying[key] = value;

						Color c = value.cast<Color>();

						// std::cout << c.red << " " << c.green << " " << c.blue << "\n";
					}

					// execute fragment shader
					Color color = fragment_shader(uniforms, interpolated_varying);

					canvas.set_pixel_color(x, y, color);
				}
			}
		}
	}
}

void draw_wireframe(Canvas canvas, const std::vector<Vector3>& vertices, const std::vector<TriangleIndices>& indices) {
	auto edges = triangles_to_edges(indices);

	for (auto edge : edges) {
		Vector3 start = vertices[edge.first];
		Vector3 end = vertices[edge.second];

		int start_x_int = static_cast<int>(std::round(start.x));
		int start_y_int = static_cast<int>(std::round(start.y));
		int end_x_int = static_cast<int>(std::round(end.x));
		int end_y_int = static_cast<int>(std::round(end.y));

		draw_line(
			canvas,
			start_x_int,
			start_y_int,
			end_x_int,
			end_y_int,
			Color(1, 0, 0)
		);
	}
}

enum class RenderOptions {
	WIREFRAME = 1 << 0,
	RASTERIZE = 1 << 1
};

inline RenderOptions operator&(RenderOptions a, RenderOptions b)
{
	return static_cast<RenderOptions>(static_cast<int>(a) & static_cast<int>(b));
}

inline RenderOptions operator|(RenderOptions a, RenderOptions b)
{
	return static_cast<RenderOptions>(static_cast<int>(a) | static_cast<int>(b));
}

template<typename U>
void render(
	Canvas canvas,
	const std::vector<Vector3>& vertices,
	const std::vector<TriangleIndices>& indices,
	std::vector<std::vector<float>>& depth_buffer,
	// const std::vector<A>& vertex_attributes,
	U uniforms,
	std::function<Vector4(Vector4, U, StringVaryingMap&)> vertex_shader,
	std::function<Color(U, const StringVaryingMap&)> fragment_shader,
	RenderOptions render_options
) {
	// execute vertex shader

	std::vector<Vector4> processed_vertices;
	std::vector<StringVaryingMap> varying;

	for (const Vector3& vertex : vertices) {
		StringVaryingMap map;
		Vector4 processed_vertex = vertex_shader(vertex.extend(1), uniforms, map);

		processed_vertices.push_back(processed_vertex);
		varying.push_back(map);
	}

	// calculate screen-space vertices

	std::vector<Vector3> screen_space_vertices;

	for (const Vector4& vertex : processed_vertices) {
		float screen_x = canvas.width * (vertex.x / vertex.w + 1) / 2;
		float screen_y = canvas.height * (vertex.y / vertex.w + 1) / 2;

		screen_space_vertices.push_back(Vector3{ screen_x, screen_y, vertex.z });
	}

	// render

	if ((render_options & RenderOptions::RASTERIZE) == RenderOptions::RASTERIZE) {
		rasterize(canvas, screen_space_vertices, indices, depth_buffer, uniforms, varying, fragment_shader);
	}

	if ((render_options & RenderOptions::WIREFRAME) == RenderOptions::WIREFRAME) {
		draw_wireframe(canvas, screen_space_vertices, indices);
	}
}

struct Uniforms {
	float time;
	Matrix4 projection;
	Vector3 position;
	Color color;
	float rotation_start;
};

Vector4 vert(Vector4 vertex, Uniforms uniforms, StringVaryingMap& varying) {
	float time = uniforms.time;
	float rs = uniforms.rotation_start;

	Matrix4 model1 = {
		1, 0, 0, 0,
		0, cos(rs + time / 2), -sin(rs + time / 2), 0,
		0, sin(rs + time / 2), cos(rs + time / 2), 0,
		0, 0, 0, 1
	};

	Matrix4 model2 = {
		cos(rs + time / 2), 0, sin(rs + time / 2), uniforms.position.x,
		0, 1, 0, uniforms.position.y,
		-sin(rs + time / 2), 0, cos(rs + time / 2), uniforms.position.z,
		0, 0, 0, 1
	};

	Matrix4 model3 = {
	1, 0, 0, uniforms.position.x,
	0, 1, 0, uniforms.position.y,
	0, 0, 1, uniforms.position.z,
	0, 0, 0, 1
	};

	Vector4 world_space_vertex = model2 * (model1 * vertex);

	if (world_space_vertex.y < 0) {
		varying["color"] = Color(0, 0, 1);
	}
	else {
		varying["color"] = Color(1, 0, 0);
	}

	return uniforms.projection * world_space_vertex;
}

Color frag(Uniforms uniforms, const StringVaryingMap& varying) {
	return varying.at("color").cast<Color>();

	// return Color(1, 0, 0);
}

void draw(void* buffer, int width, int height, float time) {
	Canvas canvas = Canvas(static_cast<uint8_t*>(buffer), width, height);
	canvas.clear(Color(0, 0, 0));

	//std::vector<Vector3> vertices{ {
	//	{0.5, -0.5, 0.0},
	//	{0.0, 0.5, 0.0},
	//	{-0.5, -0.5, 0.0}
	//} };

	//std::vector<TriangleIndices> indices{ {
	//	{0, 1, 2}
	//} };


	// Cube

	std::vector<Vector3> vertices{ {
			// front
			{ -1, -1, 1 },
			{ 1, -1, 1 },
			{ 1, 1, 1 },
			{ -1, 1, 1 },
			// back
			{ -1, -1, -1 },
			{ 1, -1, -1 },
			{ 1, 1, -1 },
			{ -1, 1, -1 }
		} };

	std::vector<TriangleIndices> indices{ {
			// front
			{0, 1, 2},
			{2, 3, 0},
			// back
			{5, 4, 7},
			{7, 6, 5},
			//left
			{4, 0, 3},
			{3, 7, 4},
			// right
			{1, 5, 6},
			{6, 2, 1},
			// top
			{3, 2, 6},
			{6, 7, 3},
			// bottom
			{4, 5, 1},
			{1, 0, 4}
	} };


	float l = -2;
	float r = 2;
	float b = -1.4f;
	float t = b + static_cast<float>(height) / static_cast<float>(width) * (r - l);

	Matrix4 projection = {
		2 / (r - l), 0, (r + l) / (r - l), 0,
		0, 2 / (t - b), (t + b) / (t - b), 0,
		0, 0, -1, 0,
		0, 0, -1, 0
	};

	Uniforms uniforms = { time, projection, {0,  2 * sin(time / 2), -4}, Color(0, 0, 1), 0};

	std::function<Vector4(Vector4, Uniforms, StringVaryingMap&)> vertex_shader = &vert;
	std::function<Color(Uniforms, const StringVaryingMap&)> fragment_shader = &frag;

	constexpr float infinity = std::numeric_limits<float>::infinity();
	std::vector<std::vector<float>> depth_buffer = std::vector(canvas.width, std::vector(canvas.height, infinity));

	render(canvas, vertices, indices, depth_buffer, uniforms, vertex_shader, fragment_shader,
		RenderOptions::RASTERIZE);

	//uniforms.position = { 0, 5 - time, -8 };
	//uniforms.color = Color(1, 0, 0);
	//uniforms.rotation_start = 1.3f;
	//uniforms.time = 2 * time;

	//render(canvas, vertices, indices, depth_buffer, uniforms, vertex_shader, fragment_shader,
	//	RenderOptions::RASTERIZE);
}