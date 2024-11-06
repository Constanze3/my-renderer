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

#include "linear_algebra.h"

struct Color {
	uint8_t red;
	uint8_t green;
	uint8_t blue;

	Color(uint8_t red, uint8_t green, uint8_t blue) : red(red), green(green), blue(blue) {}
};

using StringAnyMap = std::unordered_map<std::string, std::any>;

struct Interpolator {
	std::unordered_map <std::type_index, std::function<std::any(std::any, std::any, std::any, Vector3)>> map;

	Interpolator(bool default_values) {
		if (default_values) {
			map[typeid(Color)] = [](std::any a, std::any b, std::any c, Vector3 distribution) -> std::any {
				Color a_color = std::any_cast<Color>(a);
				Color b_color = std::any_cast<Color>(b);
				Color c_color = std::any_cast<Color>(c);

				uint8_t red = distribution.x * a_color.red + distribution.y * b_color.red + distribution.z * c_color.red;
				uint8_t green = distribution.x * a_color.green + distribution.y * b_color.green + distribution.z * c_color.green;
				uint8_t blue = distribution.x * a_color.blue + distribution.y * b_color.blue + distribution.z * c_color.blue;

				return Color(red, green, blue);
			};
		}
	}

	std::any interpolate(std::any a, std::any b, std::any c, Vector3 distribution) {
		assert(a.type() == b.type() && b.type() == c.type());

		return map.at(a.type())(a, b, c, distribution);
	}
};

void matrix_vector_multiplication(float* matrix, float* vector) {
	float out[4] = {};

	for (size_t i = 0; i < 4; i++) {
		float row_sum = 0;
		for (int j = 0; j < 4; j++) {
			row_sum += matrix[i * 4 + j] * vector[j];
		}

		out[i] = row_sum;
	}

	for (size_t i = 0; i < 4; i++) {
		vector[i] = out[i];
	}
}

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

/// <summary>
/// A buffer of BGR values.
/// The size of the buffer is 3 * width * height.
/// </summary>
struct Canvas {
	size_t width;
	size_t height;
	uint8_t* buffer;

	Canvas(uint8_t* buffer, size_t width, size_t height) : buffer(buffer), width(width), height(height) {}

	void set_pixel_color(size_t x, size_t y, Color color) const {
		assert(buffer);

		size_t index = ((height - 1 - y) * width + x) * 3;
		uint8_t* pixel = buffer + index;

		pixel[0] = color.blue;
		pixel[1] = color.green;
		pixel[2] = color.red;
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

template<typename T>
T clamp(T min, T max, T value) {
	return std::max(min, std::min(value, max));
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

	if (w1 >= 0 && w2 >= 0 && w3 >= 0) {
		// the point is inside the triangle
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
	U uniforms,
	const std::vector<StringAnyMap>& varying,
	Interpolator interpolator,
	std::function<Color(U, const StringAnyMap&)> fragment_shader
) {
	assert(0 < canvas.width);
	assert(0 < canvas.height);

	for (const TriangleIndices& triangle : indices) {
		Vector3 v1 = vertices[triangle.v1];
		Vector3 v2 = vertices[triangle.v2];
		Vector3 v3 = vertices[triangle.v3];

		size_t zero = 0;

		size_t bbmin_x = static_cast<size_t>(std::floor(std::min({ v1.x, v2.x, v3.x })));
		bbmin_x = clamp(zero, canvas.width - 1, bbmin_x);

		size_t bbmin_y = static_cast<size_t>(std::floor(std::min({ v1.y, v2.y, v3.y })));
		bbmin_y = clamp(zero, canvas.height - 1, bbmin_y);

		size_t bbmax_x = static_cast<size_t>(std::floor(std::max({ v1.x, v2.x, v3.x })));
		bbmax_x = clamp(zero, canvas.width - 1, bbmax_x);

		size_t bbmax_y = static_cast<size_t>(std::floor(std::max({ v1.y, v2.y, v3.y })));
		bbmax_y = clamp(zero, canvas.height - 1, bbmax_y);

		for (size_t x = bbmin_x; x <= bbmax_x; x++) {
			for (size_t y = bbmin_y; y <= bbmax_y; y++) {
				float x_float = static_cast<float>(x);
				float y_float = static_cast<float>(y);

				Vector3 barycentric;

				if (is_in_triangle({x_float, y_float}, v1.xy(), v2.xy(), v3.xy(), barycentric, true)) {
					// transform varying
					StringAnyMap interpolated_varying;

					StringAnyMap varying_v1 = varying[triangle.v1];
					StringAnyMap varying_v2 = varying[triangle.v2];
					StringAnyMap varying_v3 = varying[triangle.v3];

					for (auto const& item : varying_v1) {
						std::string key = item.first;

						std::any value_v1 = item.second;
						std::any value_v2 = varying_v2.at(key);
						std::any value_v3 = varying_v3.at(key);

						std::any value = interpolator.interpolate(value_v1, value_v2, value_v3, barycentric);
						interpolated_varying[item.first] = value;
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
			Color(255, 0, 0)
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
	// const std::vector<A>& vertex_attributes,
	U uniforms,
	std::function<Vector4(Vector4, U, StringAnyMap&)> vertex_shader,
	std::function<Color(U, const StringAnyMap&)> fragment_shader,
	Interpolator interpolator,
	RenderOptions render_options
) {
	// execute vertex shader

	std::vector<Vector4> processed_vertices;
	std::vector<StringAnyMap> varying;

	for (const Vector3& vertex : vertices) {
		StringAnyMap map;
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
		rasterize(canvas, screen_space_vertices, indices, uniforms, varying, interpolator, fragment_shader);
	}

	if ((render_options & RenderOptions::WIREFRAME) == RenderOptions::WIREFRAME) {
		draw_wireframe(canvas, screen_space_vertices, indices);
	}
}

struct Uniforms {
	float time;
	Matrix4 projection;
};

struct Varying { };

Vector4 vert(Vector4 vertex, Uniforms uniforms, StringAnyMap& varying) {
	float time = uniforms.time;

	Matrix4 model1 = {
		1, 0, 0, 0,
		0, cos(time / 2), -sin(time / 2), 0,
		0, sin(time / 2), cos(time / 2), 0,
		0, 0, 0, 1
	};

	Matrix4 model2 = {
		cos(time / 2), 0, sin(time / 2), 0,
		0, 1, 0, 0,
		-sin(time / 2), 0, cos(time / 2), -4,
		0, 0, 0, 1
	};

	if (vertex.x < 0) {
		varying["color"] = Color(255, 0, 0);
	}
	else if (vertex.y < 0) {
		varying["color"] = Color(0, 255, 0);
	}
	else {
		varying["color"] = Color(0, 0, 255);
	}

	return uniforms.projection * (model2 * (model1 * vertex));
}

Color frag(Uniforms uniforms, const StringAnyMap& varying) {
	return std::any_cast<Color>(varying.at("color"));
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
		0, 0, 0, 0,
		0, 0, -1, 0
	};

	Uniforms uniforms = { time, projection };
	Interpolator interpolator(true);

	std::function<Vector4(Vector4, Uniforms, StringAnyMap&)> vertex_shader = &vert;
	std::function<Color(Uniforms, const StringAnyMap&)> fragment_shader = &frag;

	render(canvas, vertices, indices, uniforms, vertex_shader, fragment_shader, interpolator, 
		RenderOptions::RASTERIZE);
}