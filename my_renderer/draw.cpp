#include <iostream>
#include <cmath>
#include <unordered_set>
#include <vector>
#include <cassert>
#include <array>

#include "linear_algebra.h"

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
	size_t p1 = 0;
	size_t p2 = 0;
	size_t p3 = 0;

	TriangleIndices(std::initializer_list<size_t> list) {
		assert(list.size() == 3);

		p1 = *list.begin();
		p2 = *(list.begin() + 1);
		p3 = *(list.begin() + 2);
	}
};

struct Color {
	uint8_t red;
	uint8_t green;
	uint8_t blue;

	Color(uint8_t red, uint8_t green, uint8_t blue) : red(red), green(green), blue(blue) {}
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

		size_t index = (y * width + x) * 3;
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

std::unordered_set<std::pair<size_t, size_t>, EdgeHasher> triangles_to_edges(const std::vector<size_t>& triangles) {
	std::unordered_set<std::pair<size_t, size_t>, EdgeHasher> edges;

	for (size_t i = 0; i < triangles.size() / 3; i++) {
		size_t v1 = triangles[3 * i];
		size_t v2 = triangles[3 * i + 1];
		size_t v3 = triangles[3 * i + 2];

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

bool edge_function(float x, float y, float x0, float y0, float x1, float y1, bool counter_clockwise) {

	if (counter_clockwise) {
		return ((x - x1) * (y0 - y1) - (x0 - x1) * (y - y1) >= 0);
	}
	else {
		return ((x - x0) * (y1 - y0) - (x1 - x0) * (y - y0) >= 0);
	}
}

bool is_in_triangle(float x, float y, float x0, float y0, float x1, float y1, float x2, float y2, bool counter_clockwise) {
	bool inside = true;
	inside &= edge_function(x, y, x0, y0, x1, y1, counter_clockwise);
	inside &= edge_function(x, y, x1, y1, x2, y2, counter_clockwise);
	inside &= edge_function(x, y, x2, y2, x0, y0, counter_clockwise);

	return inside;
}

void rasterize(Canvas canvas, const std::vector<float>& points, const std::vector<size_t>& indices) {
	assert(0 < canvas.width);
	assert(0 < canvas.height);
	assert(points.size() % 2 == 0);
	assert(indices.size() % 3 == 0);

	for (size_t i = 0; i < indices.size() / 3; i++) {
		size_t p1 = indices[3 * i];
		size_t p2 = indices[3 * i + 1];
		size_t p3 = indices[3 * i + 2];

		float x1 = points[3 * p1];
		float y1 = points[3 * p1 + 1];

		float x2 = points[3 * p2];
		float y2 = points[3 * p2 + 1];

		float x3 = points[3 * p3];
		float y3 = points[3 * p3 + 1];

		size_t zero = 0;

		size_t bbmin_x = static_cast<size_t>(std::floor(std::min({ x1, x2, x3 })));
		bbmin_x = clamp(zero, canvas.width - 1, bbmin_x);

		size_t bbmin_y = static_cast<size_t>(std::floor(std::min({ y1, y2, y3 })));
		bbmin_y = clamp(zero, canvas.height - 1, bbmin_y);
		
		size_t bbmax_x = static_cast<size_t>(std::floor(std::max({ x1, x2, x3 })));
		bbmax_x = clamp(zero, canvas.width - 1, bbmax_x);

		size_t bbmax_y = static_cast<size_t>(std::floor(std::max({ y1, y2, y3 })));
		bbmax_y = clamp(zero, canvas.height - 1, bbmax_y);

		for (size_t x = bbmin_x; x <= bbmax_x; x++) {
			for (size_t y = bbmin_y; y <= bbmax_y; y++) {
				float x_float = static_cast<float>(x);
				float y_float = static_cast<float>(y);

				if (is_in_triangle(x_float, y_float, x1, y1, x2, y2, x3, y3, true)) {
					canvas.set_pixel_color(x, y, Color(255, 255, 255));
				}
			}
		}
	}
}

void draw_wireframe(Canvas canvas, const std::vector<float>& vertices, const std::vector<size_t>& indices) {
	assert(vertices.size() % 3 == 0);

	auto edges = triangles_to_edges(indices);

	for (auto edge : edges) {
		size_t start = edge.first;
		size_t end = edge.second;

		float start_x = vertices[3 * start];
		float start_y = vertices[3 * start + 1];
		float end_x = vertices[3 * end];
		float end_y = vertices[3 * end + 1];

		int start_x_int = static_cast<int>(std::round(start_x));
		int start_y_int = static_cast<int>(std::round(start_y));
		int end_x_int = static_cast<int>(std::round(end_x));
		int end_y_int = static_cast<int>(std::round(end_y));

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

void render(Canvas canvas, const std::vector<float>& vertices, const std::vector<size_t>& indices, RenderOptions draw_options) {
	assert(vertices.size() % 4 == 0);

	std::vector<float> screen_space_vertices;

	for (size_t i = 0; i < vertices.size() / 4; i++) {
		float x = vertices[4 * i];
		float y = vertices[4 * i + 1];
		float z = vertices[4 * i + 2];
		float w = vertices[4 * i + 3];

		float screen_x = canvas.width * (x / w + 1) / 2;
		float screen_y = canvas.height * (y / w + 1) / 2;

		screen_space_vertices.push_back(screen_x);
		screen_space_vertices.push_back(screen_y);
		screen_space_vertices.push_back(z);
	}

	if ((draw_options & RenderOptions::RASTERIZE) == RenderOptions::RASTERIZE) {
		rasterize(canvas, screen_space_vertices, indices);
	}

	if ((draw_options & RenderOptions::WIREFRAME) == RenderOptions::WIREFRAME) {
		draw_wireframe(canvas, screen_space_vertices, indices);
	}
}

struct Uniforms {
	float time;
	Matrix4 projection;
};

struct Varying { };

Vector4 vertex_shader(Vector4 vector, Uniforms uniforms, Varying& out) {
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

	return uniforms.projection * (model2 * (model1 * vector));
}

Color fragment_shader(Uniforms uniforms, Varying in) {
	return Color(255, 0, 0);
}

void draw(void* buffer, int width, int height, float time) {
	auto canvas = Canvas(static_cast<uint8_t*>(buffer), width, height);
	canvas.clear(Color(0, 0, 0));

	float points[32] = { 
		// front
		-1, -1, 1, 1,
		1, -1, 1, 1,
		1, 1, 1, 1,
		-1, 1, 1, 1,
		// back
		-1, -1, -1, 1,
		1, -1, -1, 1,
		1, 1, -1, 1,
		-1, 1, -1, 1
	};

	std::array<Vector3, 8> vertices = { {
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

	std::array<size_t, 36> triangles = {
		// front
		0, 1, 2,
		2, 3, 0,
		// back
		5, 4, 7,
		7, 6, 5,
		//left
		4, 0, 3,
		3, 7, 4,
		// right
		1, 5, 6,
		6, 2, 1,
		// top
		3, 2, 6,
		6, 7, 3,
		// bottom
		4, 5, 1,
		1, 0, 4
	};

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

	std::vector<float> points_vec(points, points + 32);
	std::vector<size_t> indices_vec(triangles.begin(), triangles.end());

	Uniforms uniforms = {time, projection};

	for (size_t i = 0; i < 8; i++) {
		Varying varying;
		vertex_shader(vertices[i].extend(1), uniforms, varying);
	}

	render(canvas, points_vec, indices_vec, RenderOptions::WIREFRAME | RenderOptions::RASTERIZE);
}