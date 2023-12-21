#include <iostream>
#include <cmath>
#include <SFML/Graphics.hpp>

using namespace std;

sf::RenderWindow window(sf::VideoMode(1920,1080), "Tree");

void start(float inix, float iniy, float degree, float length)
{
	if (length < 1.0f) return;
	
	else
	{
		float newlength=0.67f * length;
		
		float finx = inix + (newlength * (float)cos(degree * (3.14 / 180.0f)));
		float finy = iniy + (newlength * (float)sin(degree * (3.14 / 180.0f)));
		
		sf::Vertex line[]= {
				sf::Vertex(sf::Vector2f(inix, iniy)),
				sf::Vertex(sf::Vector2f(finx, finy))
		};
		start(finx, finy, degree+60.0f, newlength);
		start(finx, finy, degree-30.0f, newlength);
		
		window.draw(line, 2, sf::Lines);
		//line.setFillColor(sf::Color::Blue);
		//window.display(); // Uncomment for animation
	}
}


int main()
{
	window.clear();
	start(1000, 799, -90, 350);
	window.display();
	
	sf::Event event;
	while (window.isOpen())
	{
		while (window.pollEvent(event))
		{
			if (event.type == sf::Event::Closed) window.close();
		}
	}
	
	return 0;
}

