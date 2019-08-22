#include "Fish.hpp"
#include "Tools.hpp"
#include "Environment.hpp"

Fish::Fish(sf::Vector2f position) {
	float radius = 20;
	shape.setFillColor(sf::Color::Red);
	shape.setRadius(radius);
	shape.setOrigin(shape.getLocalBounds().width / 2, shape.getLocalBounds().height / 2);

	float invMass = 1 / (M_PI * ipowf(radius, 2) * SURFACICMASS);
	invMasses.diagonal() << invMass, invMass;
	invInertias.diagonal() << invMass * 1 / (0.5f * ipowf(radius, 2));
	forces << 0, 0;
	torques << 0;
	speeds << 0, 0;
	rotSpeeds << 0;
	positions << position.x, position.y;
	rotations << 0;

	updateGraphics();
}

uint Fish::addArm(float length, uint parentArmID, sf::Vector2f jointPos_parent, sf::Vector2f jointPos_child, float initialOrientation) {
	std::shared_ptr<Arm> newArm = nullptr;
	size_t parentIndex;

	if (parentArmID == 0) {
		children.push_back(std::make_shared<Arm>(nullptr, nextArmID, length));
		newArm = children.back();
		parentIndex = 0;
	}
	else {
		auto parentIt = arms.find(parentArmID);
		if (parentIt == arms.end()) {
			throw "Trying to attach arm to nonexistent parent";
		}

		std::shared_ptr parent = parentIt->second;
		
		parent->children.push_back(std::make_shared<Arm>(parent, nextArmID, length));
		newArm = parent->children.back();
		parentIndex = parent->matIndex;
	}

	arms[nextArmID] = newArm;

	float mass = length * ARMWIDTH * SURFACICMASS;

	size_t nB = arms.size() + 1;
	newArm->matIndex = nB - 1;
	size_t nL = nB - 1;
	invMasses.diagonal().conservativeResize(2 * nB);
	invMasses.diagonal().tail(2).fill(1 / mass);

	invInertias.diagonal().conservativeResize(nB);
	invInertias.diagonal().tail(1) << 1 / ((ipowf(length, 2) + ipowf(ARMWIDTH, 2)) * mass / 12);

	localJointPos_child.conservativeResize(2 * nL);
	localJointPos_child.tail(2) << jointPos_child.x, jointPos_child.y;

	localJointPos_parent.conservativeResize(2 * nL);
	localJointPos_parent.tail(2) << jointPos_parent.x, jointPos_parent.y;

	forces.conservativeResize(2 * nB);
	forces.tail(2) << 0, 0;

	torques.conservativeResize(nB);
	torques.tail(1) << 0;

	speeds.conservativeResize(2 * nB);
	speeds.tail(2) << 0, 0;

	rotSpeeds.conservativeResize(nB);
	rotSpeeds.tail(1) << 0;

	rotations.conservativeResize(nB);
	rotations.tail(1) << (float)M_PI / 180.f * initialOrientation;

	Eigen::Vector2f newPos = positions.segment(parentIndex * 2, 2)
							 + Eigen::Rotation2D<float>(rotations(parentIndex)).toRotationMatrix() * localJointPos_parent.tail(2)
							 + Eigen::Rotation2D<float>(rotations.tail(1)(0)).toRotationMatrix() * (-localJointPos_child.tail(2));

	positions.conservativeResize(2 * nB);
	positions.tail(2) << newPos;

	return nextArmID++;
}

void Fish::updatePhysics(float deltaT) {
	size_t nB = arms.size() + 1;
	size_t nL = nB - 1;
	Eigen::MatrixXf A(2*nL, 2*nL);
	A.fill(0);
	Eigen::VectorXf b(2*nL);
	b.fill(0);

	//Pre-calculate global joint positions
	Eigen::Matrix2f* rotationMatrices = new Eigen::Matrix2f[nB];
	for (size_t i = 0; i < nB; i++) {
		rotationMatrices[i] = Eigen::Rotation2D<float>(rotations(i)).toRotationMatrix();
	}
	Eigen::VectorXf jointPos_child(2*nL);
	Eigen::VectorXf jointPos_parent(2*nL);
	for (auto it = arms.begin(); it != arms.end(); it++) {
		uint i = it->second->matIndex;

		uint j = 0;
		if (it->second->parent.get() != nullptr)
			j = it->second->parent->matIndex;

		jointPos_child.segment(2*(i-1), 2) = rotationMatrices[i] * localJointPos_child.segment(2*(i-1), 2); 
		jointPos_parent.segment(2*(i-1), 2) = rotationMatrices[j] * localJointPos_parent.segment(2*(i-1), 2);		
	}
	delete[] rotationMatrices;

	for (auto it = arms.begin(); it != arms.end(); it++) {
		Arm& c = *(it->second);
		uint i = c.matIndex;
		
		uint j = 0;
		if (it->second->parent.get() != nullptr)
			j = it->second->parent->matIndex;

		Eigen::Vector2f dij = jointPos_child.segment(2*(i-1), 2);
		Eigen::Vector2f dji = jointPos_parent.segment(2*(i-1), 2);

		float imi = invMasses.diagonal()(2*i);
		float imj = invMasses.diagonal()(2*j);
		float iIi = invInertias.diagonal()(i);
		float iIj = invInertias.diagonal()(j);

		//j->i
		//x-equations
		//j->i | x
		A(2*(i-1), 2*(i-1)) += imi - iIi * dij.y() * (-dij.y());
		//j->i | y
		A(2*(i-1), 2*(i-1)+1) += iIi * dij.y() * (-dij.x());

		//y-equations
		A(2*(i-1)+1, 2*(i-1)) += iIi * dij.x() * (-dij.y());
		A(2*(i-1)+1, 2*(i-1)+1) += imi - iIi * dij.x() * (-dij.x());

		//pj->j
		if (j != 0) {
			Eigen::Vector2f djpj = jointPos_child.segment(2*(j-1), 2);

			A(2*(i-1), 2*(j-1)) -= imj - iIj * dji.y() * (-djpj.y());
			A(2*(i-1), 2*(j-1)+1) -= iIj * dji.y() * (-djpj.x());
			A(2*(i-1)+1, 2*(j-1)) -= iIj * dji.x() * (-djpj.y());
			A(2*(i-1)+1, 2*(j-1)+1) -= imj - iIj * dji.x() * (-djpj.x());
		}

		//i->ci
		for (auto it2 = c.children.begin(); it2 != c.children.end(); it2++) {
			uint k = (*it2)->matIndex;
			Eigen::Vector2f dik = jointPos_parent.segment(2*(k-1), 2);

			A(2*(i-1), 2*(k-1)) -= imi - iIi * dij.y() * (-dik.y());
			A(2*(i-1), 2*(k-1)+1) -= iIi * dij.y() * (-dik.x());

			A(2*(i-1)+1, 2*(k-1)) -= iIi * dij.x() * (-dik.y());
			A(2*(i-1)+1, 2*(k-1)+1) -= imi - iIi * dij.x() * (-dik.x());
		}

		std::vector<std::shared_ptr<Arm>>* j_children;
		if (j == 0)
			j_children = &children;
		else
			j_children = &c.parent->children;

		//j->cj (including i->j)
		for (auto it2 = j_children->begin(); it2 != j_children->end(); it2++) {
			uint k = (*it2)->matIndex;
			Eigen::Vector2f djk = jointPos_parent.segment(2*(k-1), 2);

			A(2*(i-1), 2*(k-1)) += imj - iIj * dji.y() * (-djk.y());
			A(2*(i-1), 2*(k-1)+1) += iIj * dji.y() * (-djk.x());

			A(2*(i-1)+1, 2*(k-1)) += iIj * dji.x() * (-djk.y());
			A(2*(i-1)+1, 2*(k-1)+1) += imj - iIj * dji.x() * (-djk.x());
		}

		b(2*(i-1)) -= forces(2*i) * imi - forces(2*j) * imj
					- dij.y() * torques(i) * iIi - dij.x() * ipowf(rotSpeeds(i), 2)
					+ dji.y() * torques(j) * iIj + dji.x() * ipowf(rotSpeeds(j), 2);
		b(2*(i-1)+1) -= forces(2*i+1) * imi - forces(2*j+1) * imj
					+ dij.x() * torques(i) * iIi - dij.y() * ipowf(rotSpeeds(i), 2)
					- dji.x() * torques(j) * iIj + dji.y() * ipowf(rotSpeeds(j), 2);

	}

	Eigen::VectorXf fp = A.colPivHouseholderQr().solve(b);

	Eigen::VectorXf fl(2 * nB);
	fl.fill(0);

	Eigen::VectorXf tl(nB);
	tl.fill(0);

	for (auto it = arms.begin(); it != arms.end(); it++) {
		uint i = it->second->matIndex;

		uint j = 0;
		if (it->second->parent.get() != nullptr)
			j = it->second->parent->matIndex;

		Eigen::Vector2f f_j_to_i = fp.segment(2*(i-1), 2);
		fl.segment(2*i, 2) += f_j_to_i;
		fl.segment(2*j, 2) -= f_j_to_i;

		Eigen::Vector2f dij = jointPos_child.segment(2*(i-1), 2);
		Eigen::Vector2f dji = jointPos_parent.segment(2*(i-1), 2);

		tl(i) += cross(dij, f_j_to_i);
		tl(j) -= cross(dji, f_j_to_i);
	}

	speeds += deltaT * invMasses * (fl + forces);
	rotSpeeds += deltaT * invInertias * (tl + torques);
	positions += deltaT * speeds;
	rotations += deltaT * rotSpeeds;

	//Clamping
	//Recalculate global joint positions
	rotationMatrices = new Eigen::Matrix2f[nB];
	for (size_t i = 0; i < nB; i++) {
		rotationMatrices[i] = Eigen::Rotation2D<float>(rotations(i)).toRotationMatrix();
	}
	for (auto it = arms.begin(); it != arms.end(); it++) {
		uint i = it->second->matIndex;

		uint j = 0;
		if (it->second->parent.get() != nullptr)
			j = it->second->parent->matIndex;

		jointPos_child.segment(2*(i-1), 2) = rotationMatrices[i] * localJointPos_child.segment(2*(i-1), 2); 
		jointPos_parent.segment(2*(i-1), 2) = rotationMatrices[j] * localJointPos_parent.segment(2*(i-1), 2);		
	}
	delete[] rotationMatrices;

	std::function<void(Arm&, uint)> recursiveClamp = [&](Arm& arm, uint j){
		uint i = arm.matIndex;
		speeds.segment(2*i, 2) = speeds.segment(2*j, 2) + cross(rotSpeeds(j), jointPos_parent.segment(2*(i-1), 2))
														- cross(rotSpeeds(i), jointPos_child.segment(2*(i-1), 2));
		positions.segment(2*i, 2) = positions.segment(2*j, 2) + jointPos_parent.segment(2*(i-1), 2)
															  - jointPos_child.segment(2*(i-1), 2);
		for (auto it = arm.children.begin(); it != arm.children.end(); it++) {
			recursiveClamp(**it, i);
		}
	};

	for (auto it = children.begin(); it != children.end(); it++) {
		recursiveClamp(**it, 0);
	}
}

void Fish::updateGraphics() {
	shape.setPosition(sf::Vector2f(positions(0), positions(1)));
	shape.setRotation(180.f / M_PI * rotations(0));

	for (auto it = arms.begin(); it != arms.end(); it++) {
		int i = it->second->matIndex;
		it->second->setPosition(sf::Vector2f(positions(2*i), positions(2*i+1)));
		it->second->setRotation(180.f / M_PI * rotations(i));
	}

}

void Fish::draw(sf::RenderTarget& target, sf::RenderStates states) const {
	for (auto it = children.begin(); it != children.end(); it++) {
		target.draw(**it, states);
	};

	target.draw(shape, states);
}