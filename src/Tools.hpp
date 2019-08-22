#ifndef TOOLS_H
#define TOOLS_H

float norm(sf::Vector2f const& v);

Eigen::Vector2f cross(Eigen::Vector2f v, float f);
Eigen::Vector2f cross(float f, Eigen::Vector2f v);
float cross(Eigen::Vector2f l, Eigen::Vector2f r);

float ipowf(float f, int i);

float randf();

float randf(float max);

float randf(float min, float max);

sf::Vector2f randomPos(sf::FloatRect const& playArea);

ulong getUniqueNumber();

#endif