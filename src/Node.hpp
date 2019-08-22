#ifndef NODE_H
#define NODE_H

class Node : public sf::Drawable, public sf::Transformable {
public:
	Node() {
		shape.setFillColor(sf::Color::Yellow);
		shape.setRadius(10);
		shape.setOrigin(shape.getLocalBounds().width / 2, shape.getLocalBounds().height / 2);
	}

private:
	sf::CircleShape shape;
	virtual void draw(sf::RenderTarget& target, sf::RenderStates states) const {
		states.transform *= getTransform();
		target.draw(shape, states);
	}
};

#endif