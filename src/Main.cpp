#include "Main.hpp"
#include "Fish.hpp"
#include "Tools.hpp"

int main()
{
	srand (time(NULL));

	sf::RenderWindow window(sf::VideoMode(1400, 800), "Aquarium", sf::Style::Close);
#ifdef SFML_SYSTEM_WINDOWS
	__windowsHelper.setIcon(window.getSystemHandle());
#endif
	window.setFramerateLimit(60);

	sf::Event haps;

	sf::Clock clock;
	clock.restart();

	Fish fish(sf::Vector2f(700, 400));
	uint left = fish.addArm(100, 0, sf::Vector2f(-20, 0), sf::Vector2f(-50, 0), 90);
	uint right = fish.addArm(100, 0, sf::Vector2f(20, 0), sf::Vector2f(-50, 0), 0);
	fish.addArm(100, left, sf::Vector2f(50, 0), sf::Vector2f(-50, 0), 180);

	bool paused = true;

	while (window.isOpen())
	{
		while (window.pollEvent(haps))
		{
			if (haps.type == sf::Event::Closed)
				window.close();
			else if (haps.type == sf::Event::KeyPressed) {
				if (haps.key.code == sf::Keyboard::Space) {
					paused = !paused;
				}
			}
		}

		clock.restart();
		
		if (sf::Mouse::isButtonPressed(sf::Mouse::Left)) {
			sf::Vector2f mouseDist = (sf::Vector2f)sf::Mouse::getPosition(window) - fish.getPosition();
			fish.setForce(0, 500.f * mouseDist / norm(mouseDist));
		}
		else {
			fish.setForce(0, sf::Vector2f(0, 0));
		}

		if (!paused)
			fish.updatePhysics(0.02);

		window.clear(sf::Color::Magenta);
		fish.updateGraphics();
		window.draw(fish);
		window.display();
	}

	return 0;
}
