#ifndef FISH_H
#define FISH_H

#include "Arm.hpp"
#include <eigen3/Eigen/Core>

class Fish : public sf::Drawable {
public:
	Fish(sf::Vector2f position = sf::Vector2f(0, 0));

	uint addArm(float length, uint parentArmID, sf::Vector2f jointPos_parent, sf::Vector2f jointPos_child, float initialOrientation);

	void updatePhysics(float deltaT);
	void updateGraphics();

	void setForce(uint id, sf::Vector2f force) {
		forces.segment(2*id, 2) << force.x, force.y;
	}

	void setTorque(uint id, float torque) {
		torques(id) = torque;
	}

	sf::Vector2f getPosition() const {
		return sf::Vector2f(positions(0), positions(1));
	}

private:
	Eigen::DiagonalMatrix<float, Eigen::Dynamic> invMasses{2};		//2nB*2nB, cst
	Eigen::DiagonalMatrix<float, Eigen::Dynamic> invInertias{1};	//nB*nB, cst
	Eigen::VectorXf localJointPos_parent{0};	//2nL*1, cst
	Eigen::VectorXf localJointPos_child{0};		//2nL*1, cst
	Eigen::VectorXf forces{2};		//2nB*1
	Eigen::VectorXf torques{1};		//nB*1
	Eigen::VectorXf speeds{2};		//2nB*1
	Eigen::VectorXf rotSpeeds{1};	//nB*1
	Eigen::VectorXf positions{2};	//2nB*1
	Eigen::VectorXf rotations{1};	//nB*1

	std::map<uint, std::shared_ptr<Arm>> arms;
	uint nextArmID = 1;

	sf::CircleShape shape;
	std::vector<std::shared_ptr<Arm>> children;

	virtual void draw(sf::RenderTarget& target, sf::RenderStates states) const;
};

#endif