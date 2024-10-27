#include <iostream>
#include <cmath>
#include <unordered_set>
#include <vector>
#include <cassert>
#include <array>

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
};

void set_pixel_color(void* buffer, int width, int height, int x, int y, Color color) {
	if (!buffer) return;

	int index = (y * width + x) * 3;
	uint8_t* pPixel = static_cast<uint8_t*>(buffer) + index;

	pPixel[0] = color.blue;
	pPixel[1] = color.green;
	pPixel[2] = color.red;
}

float lerp(float start, float end, float t) {
	return start * (1 - t) + end * t;
}

void draw_line(void* buffer, int width, int height, int startX, int startY, int endX, int endY, Color color) {
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

	for (int x = startX; x <= endX && 0 <= x && x < width; x++) {
		if (0 <= y && y < height) {
			if (!inverted) {
				set_pixel_color(buffer, width, height, x, y, color);
			}
			else {
				set_pixel_color(buffer, height, width, y, x, color);
			}
		}

		if (d > 0) {
			d -= 2 * dx;
			y += yi;
		}
		d += 2 * dy;
	}
}

void matrix_vector_multiplication(float matrix[4][4], float* vector) {
	float out[4] = {};

	for (int i = 0; i < 4; i++) {
		float row_sum = 0;
		for (int j = 0; j < 4; j++) {
			row_sum += matrix[i][j] * vector[j];
		}

		out[i] = row_sum;
	}

	for (int i = 0; i < 4; i++) {
		vector[i] = out[i];
	}
}

struct EdgeHasher {
	inline size_t operator()(const std::pair<int, int>& p) const {
		std::hash<int> hasher;
		return hasher(p.first + p.second);
	}
};

template<size_t S>
std::unordered_set<std::pair<int, int>, EdgeHasher> triangles_to_edges(std::array<size_t, S>& triangles) {
	std::unordered_set<std::pair<int, int>, EdgeHasher> edges;

	for (int i = 0; i < S / 3; i++) {
		int t = i * 3;

		int v1 = triangles[t];
		int v2 = triangles[t + 1];
		int v3 = triangles[t + 2];

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

/// <summary>
/// Rasterizes the triangles expressed with vertices and indices to a canvas.
/// 
/// </summary>
/// <param name="canvas">The canvas to rasterize to.</param>
/// <param name="points">A vector in which each 2 elements are the coordinates of a point on the canvas.</param>
/// <param name="indices">A vector in which each 3 indices are indices of points that form a triangle.</param>
void rasterize(Canvas canvas, std::vector<float> points, std::vector<size_t> indices) {
	assert(0 < canvas.width);
	assert(0 < canvas.height);
	assert(points.size() % 2 == 0);
	assert(indices.size() % 3 == 0);

	for (size_t i = 0; i < indices.size() / 3; i++) {
		size_t p1 = indices[3 * i];
		size_t p2 = indices[3 * i + 1];
		size_t p3 = indices[3 * i + 2];

		float x1 = points[2 * p1];
		float y1 = points[2 * p1 + 1];

		float x2 = points[2 * p2];
		float y2 = points[2 * p2 + 1];

		float x3 = points[2 * p3];
		float y3 = points[2 * p3 + 1];

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

void render_once(void* buffer, int width, int height) {
	// draw_line(buffer, width, height, 100, 410, 300, 100);

}

void render(void* buffer, int width, int height, float time) {

	
	for (int x = 0; x < width; x++) {
		for (int y = 0; y < height; y++) {
			set_pixel_color(buffer, width, height, x, y, Color(0, 0, 0));
		}
	}
	

	/*
	for (int x = 0; x < width; x++) {
		for (int y = 0; y < height; y++) {
			uint8_t red = static_cast<uint8_t>(lerp(0, 255, abs(sin(time))));
			uint8_t green = static_cast<uint8_t>(lerp(0, 255, static_cast<float>(x) / static_cast<float>(width)));
			uint8_t blue = static_cast<uint8_t>(lerp(0, 255, static_cast<float>(y) / static_cast<float>(height)));

			set_pixel_color(buffer, width, height, x, y, Color(red, green, blue));
		}
	}
	*/

	float points[8][4] = { 
		// front
		{-1, -1, 1, 1},
		{1, -1, 1, 1},
		{1, 1, 1, 1},
		{-1, 1, 1, 1},
		// back
		{-1, -1, -1, 1},
		{1, -1, -1, 1},
		{1, 1, -1, 1},
		{-1, 1, -1, 1}
	};

	int nice_edges[12][2] = {
		// front
		{0, 1},
		{1, 2},
		{2, 3},
		{3, 0},
		// back
		{4, 5},
		{5, 6},
		{6, 7},
		{7, 4},
		// sides
		{0, 4},
		{1, 5},
		{2, 6},
		{3, 7}
	};

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

	float model1[4][4] = {
		1, 0, 0, 0,
		0, cos(time / 2), -sin(time / 2), 0,
		0, sin(time / 2), cos(time / 2), 0,
		0, 0, 0, 1
	};


	float model2[4][4] = {
		cos(time / 2), 0, sin(time / 2), 0,
		0, 1, 0, 0,
		-sin(time / 2), 0, cos(time / 2), -4,
		0, 0, 0, 1
	};

	float l = -2;
	float r = 2;
	float b = -1.4f;
	float t = b + static_cast<float>(height) / static_cast<float>(width) * (r - l);

	float projection[4][4] = {
		2 / (r - l), 0, (r + l) / (r - l), 0,
		0, 2 / (t - b), (t + b) / (t - b), 0,
		0, 0, 0, 0,
		0, 0, -1, 0
	};

	float transformed_points[16];
	for (int i = 0; i < 8; i++) {
		matrix_vector_multiplication(model1, points[i]);
		matrix_vector_multiplication(model2, points[i]);
		matrix_vector_multiplication(projection, points[i]);

		// std::cout << "x: " << points[i][0] << " y: " << points[i][1] << " z: " << points[i][2] << "\n";

		transformed_points[2 * i] = width * (1 + points[i][0] / points[i][3]) / 2;
		transformed_points[2 * i + 1] = height * (1 + points[i][1] / points[i][3]) / 2;

		// std::cout << "x: " << transformed_points[i][0] << " y: " << transformed_points[i][1] << "\n";
	}



	
	auto canvas = Canvas(static_cast<uint8_t*>(buffer), width, height);

	std::vector<float> points_vec(transformed_points, transformed_points + 16);
	std::vector<size_t> indices_vec(triangles.begin(), triangles.end());

	
	rasterize(canvas, points_vec, indices_vec);

	/*
	auto edges = triangles_to_edges(triangles);

	for (auto edge : edges) {
		int start = edge.first;
		int end = edge.second;

		int startX = transformed_points[2 * start];
		int startY = transformed_points[2 * start + 1];
		int endX = transformed_points[2 * end + 0];
		int endY = transformed_points[2 * end + 1];

		draw_line(buffer, width, height, startX, startY, endX, endY, Color(255, 0, 0));
	}
	*/

	
	/*
	for (int i = 0; i < 12; i++) {
		int start = nice_edges[i][0];
		int end = nice_edges[i][1];

		int startX = transformed_points[2 * start];
		int startY = transformed_points[2 * start + 1];
		int endX = transformed_points[2 * end];
		int endY = transformed_points[2 * end + 1];

		// std::cout << "x0: " << startX << " y0: " << startY << " x1: " << endX << " y1: " << endY << "\n";

		draw_line(buffer, width, height, startX, startY, endX, endY, Color(0, 0, 255));
	}
	*/
	
	// colorful stuff
	/*
	for (int x = 0; x < width; x++) {
		for (int y = 0; y < height; y++) {
			uint8_t red = static_cast<uint8_t>(lerp(0, 255, abs(sin(time))));
			uint8_t green = static_cast<uint8_t>(lerp(0, 255, static_cast<float>(x) / static_cast<float>(width)));
			uint8_t blue= static_cast<uint8_t>(lerp(0, 255, static_cast<float>(y) / static_cast<float>(height)));
			
			set_pixel_color(buffer, width, height, x, y, red, green, blue);
		}
	}
	*/
}