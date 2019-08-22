#include "Arm.hpp"
#include "Tools.hpp"

//mass(length * 20), inertiaMoment()
Arm::Arm(std::shared_ptr<Arm> parent, uint ID, float length) : parent(parent), ID(ID) {
	shape.setFillColor(sf::Color::Blue);
	shape.setSize(sf::Vector2f(length, 20));
	shape.setOrigin(shape.getLocalBounds().width / 2, shape.getLocalBounds().height / 2);
}

void Arm::draw(sf::RenderTarget& target, sf::RenderStates states) const {
	for (auto it = children.begin(); it != children.end(); it++)
		target.draw(**it, states);
	states.transform *= getTransform();
	target.draw(shape, states);
}