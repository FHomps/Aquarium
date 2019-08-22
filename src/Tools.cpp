#include "Tools.hpp"

ulong nextUniqueNumberToGive = 0;

float norm(sf::Vector2f const& v) {
	return sqrtf(ipowf(v.x, 2) + ipowf(v.y, 2));
}

Eigen::Vector2f cross(Eigen::Vector2f v, float f) {
	return Eigen::Vector2f(v.y() * f, -v.x() * f);
}

Eigen::Vector2f cross(float f, Eigen::Vector2f v) {
	return Eigen::Vector2f(-v.y() * f, v.x() * f);
}

float cross(Eigen::Vector2f l, Eigen::Vector2f r) {
	return l.x() * r.y() - l.y() * r.x();
}

float ipowf(float f, int i) {
	if (i == 0)
		return 1.f;
	else if (i == 1)
		return f;
	return f * ipowf(f, i-1);
}

float randf() {
	return static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
}

float randf(float max) {
	return static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / max));
}

float randf(float min, float max) {
	return min + static_cast<float>(rand()) /( static_cast<float>(RAND_MAX/(max-min)));
}

sf::Vector2f randomPos(sf::FloatRect const& playArea) {
	return sf::Vector2f(randf(playArea.left, playArea.left + playArea.width), randf(playArea.top, playArea.top + playArea.height));
}

ulong getUniqueNumber() {
	return nextUniqueNumberToGive++;
}