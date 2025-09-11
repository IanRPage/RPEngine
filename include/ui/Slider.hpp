#ifndef SLIDER_H
#define SLIDER_H

#include <SFML/Graphics.hpp>

class HorizSlider {
private:
  sf::Vector2f position;
  float width;
  sf::Vector2f range;
  float &val;
  sf::RectangleShape bar;
  sf::CircleShape handle;

public:
  bool isDragging;

  HorizSlider(sf::Vector2f pos, sf::Vector2f size, sf::Vector2f range,
              float &val, sf::Color barColor = sf::Color::White,
              sf::Color handleColor = sf::Color::Cyan)
      : position(pos), width(size.x), range(range), val(val), bar(size),
        handle(size.y * 1.5), isDragging(false) {
    bar.setFillColor(barColor);
    handle.setFillColor(handleColor);
    bar.setPosition(position);
    handle.setPosition({position.x - handle.getRadius(), position.y - 10.0f});
    val = range.x;
  };

  void setPosition(sf::Vector2f pos) {
    position = pos;
    bar.setPosition(position);

    const float rad = handle.getRadius();
    const float t = (val - range.x) / (val - range.y);
    float mx = position.x - rad + t * width;
    handle.setPosition({mx, position.y - 10.0f});
  }

  void setSize(sf::Vector2f size) {
    width = size.x;
    bar.setSize(size);
    setPosition(position);
  }

  void draw(sf::RenderWindow &window) {
    window.draw(bar);
    window.draw(handle);
  }

  bool contains(sf::Vector2i pos) {
    return handle.getGlobalBounds().contains(static_cast<sf::Vector2f>(pos));
  }

  void move(sf::Vector2i pos) {
    auto [mx, my] = pos;
    float radHandle = handle.getRadius();
    if (mx > position.x + width - radHandle) {
      mx = position.x + width - radHandle;
    } else if (mx < position.x - radHandle) {
      mx = position.x - radHandle;
    }
    handle.setPosition({static_cast<float>(mx), handle.getPosition().y});
    val =
        range.x + ((mx - position.x + radHandle) / width) * (range.y - range.x);
  }
};

#endif
