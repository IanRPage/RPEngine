#ifndef SLIDER_H
#define SLIDER_H

#include <SFML/Graphics.hpp>
#include <algorithm>

class HorizSlider {
public:
  bool isDragging = false;

  HorizSlider(sf::Vector2f pos, sf::Vector2f size, sf::Vector2f range,
              float &val, sf::Color barColor = sf::Color::White,
              sf::Color handleColor = sf::Color::Cyan)
      : position_(pos), width_(size.x), range_(range), val_(val) {
    bar_.setFillColor(barColor);
    syncBar();

    const float handleRad = std::max(height_ * 0.9f, 0.6f);
    handle_.setFillColor(handleColor);
    handle_.setRadius(handleRad);
    handle_.setOrigin({handleRad, handleRad});

    val_ = (range_.x + range_.y) * 0.5f;
    syncHandleFromValue(); // starts in the middle b/c of line above
  };

  void setPosition(sf::Vector2f pos) {
    position_ = pos;
    syncBar();
    syncHandleFromValue();
  }

  void setSize(sf::Vector2f size) {
    width_ = size.x;
    height_ = size.y;
    syncBar();

    const float handleRad = std::max(height_ * 0.9f, 6.0f);
    handle_.setRadius(handleRad);
    handle_.setOrigin({handleRad, handleRad});
    syncHandleFromValue();
  }

  bool contains(sf::Vector2i pos) {
    return handle_.getGlobalBounds().contains(static_cast<sf::Vector2f>(pos));
  }

  void move(sf::Vector2i pos) {
		float x = static_cast<float>(pos.x);
		x = std::clamp(x, position_.x, position_.x + width_);
		
		val_ = clampToRange(xToValue(x));
		syncHandleFromValue();
  }

  void draw(sf::RenderWindow &window) {
    window.draw(bar_);
    window.draw(handle_);
  }

private:
  // geometry & vals
  sf::Vector2f position_{0.0f, 0.0f};
  float width_{0.0f};
  float height_{0.0f};
  sf::Vector2f range_{0.0f, 1.0f};
  float &val_;

  // visuals
  sf::RectangleShape bar_;
  sf::CircleShape handle_;

  // helpers
  float clampToRange(float v) { return std::clamp(v, range_.x, range_.y); }

  float valueToX(float v) {
    const float t = (v - range_.x) / (range_.y - range_.x);
    return position_.x + t * width_;
  };

  float xToValue(float x) {
    const float t = (x - position_.x) / width_;
    return range_.x + t * (range_.y - range_.x);
  };

  void syncBar() {
    bar_.setPosition(position_);
    bar_.setSize({width_, height_});
  }

  void syncHandleFromValue() {
    const float x = valueToX(clampToRange(val_));
    handle_.setPosition({x, position_.y + height_ * 0.5f});
  }
};

#endif
