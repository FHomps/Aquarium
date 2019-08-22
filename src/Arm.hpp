#ifndef ARM_H
#define ARM_H

class Arm : public sf::Drawable, public sf::Transformable {
public:
	Arm(std::shared_ptr<Arm> parent, uint ID, float length = 100);

	std::shared_ptr<Arm> parent;
	std::vector <std::shared_ptr<Arm>> children;

	const uint ID;
	uint matIndex;

private:
	sf::RectangleShape shape;
	virtual void draw(sf::RenderTarget& target, sf::RenderStates states) const;
};

#endif