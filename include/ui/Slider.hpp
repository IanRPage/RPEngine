#ifndef SLIDER_H
#define SLIDER_H

#include <SFML/Graphics.hpp>
#include <algorithm>

class HorizSlider {
private:
  // geometry & model
  sf::Vector2f position_{0.f, 0.f}; // top-left of the track
  float width_{0.f};                // track width
  float height_{10.f};              // track height (visual thickness)
  sf::Vector2f range_{0.f, 1.f};    // [min, max] value range
  float &val_;                      // external value (reference)

  // visuals
  // layered track (background + active fill)
  sf::RectangleShape trackBg_;
  sf::RectangleShape trackFill_;

  sf::RectangleShape bar_;
  sf::CircleShape handle_;

  struct Theme {
    sf::Color trackBg = sf::Color(40, 40, 48);        // dark base
    sf::Color trackFill = sf::Color(120, 170, 255);   // accent
    sf::Color handle = sf::Color::Cyan;               // default knob
    sf::Color handleHover = sf::Color(255, 235, 120); // hover
    sf::Color handleActive = sf::Color(255, 210, 90); // dragging
  } theme_;

  // text
  const sf::Font &font_;
  std::string name_;
  sf::Text text_;
  std::ostringstream oss;

  // helpers
  float clampToRange(float v) const {
    return std::clamp(v, range_.x, range_.y);
  }

  float valueToX(float v) const {
    const float t = (v - range_.x) / (range_.y - range_.x);
    return position_.x + t * width_;
  }

  float xToValue(float x) const {
    const float t = (x - position_.x) / width_;
    return range_.x + t * (range_.y - range_.x);
  }

  void syncTrack() {
    // track (bg + fill) positions/sizes
    trackBg_.setPosition(position_);
    trackBg_.setSize({width_, height_});

    // fill starts at left; width updated in syncHandleFromValue()
    trackFill_.setPosition(position_);
    // size.x set in syncHandleFromValue()
    trackFill_.setSize({0.f, height_});

    // optional outline rectangle (can be removed; keep for debug)
    bar_.setPosition(position_);
    bar_.setSize({width_, height_});
    bar_.setFillColor(sf::Color::Transparent);
    bar_.setOutlineColor(sf::Color(255, 255, 255, 30));
    bar_.setOutlineThickness(0.1f); // set >0 if you want a thin outline
  }

  void syncHandleFromValue() {
    const float cx = std::clamp(valueToX(clampToRange(val_)), position_.x,
                                position_.x + width_);
    const float cy = position_.y + height_ * 0.5f;

    // track fill extends from left edge to current x
    trackFill_.setSize({cx - position_.x, height_});

    // handle centered at (cx, cy)
    handle_.setPosition({cx, cy});
  }

  void setHandleRadiusFromHeight() {
    const float handleRadius = std::max(height_ * 0.9f, 6.0f);
    handle_.setRadius(handleRadius);
    handle_.setOrigin({handleRadius, handleRadius});
  }

  void updateText() {
    oss.str("");
    oss << name_ + ": " << val_;
    text_.setString(oss.str());
  }

public:
  bool isDragging = false;

  HorizSlider(sf::Vector2f pos, sf::Vector2f size, sf::Vector2f range,
              float &valRef, const sf::Font &font, std::string name,
              sf::Color trackBg = sf::Color::White,
              sf::Color handleColor = sf::Color::Cyan)
      : position_(pos), width_(size.x), height_(size.y), range_(range),
        val_(valRef), font_(font), name_(name), text_(font_, name) {

    // apply theme (respect legacy ctor colors where provided)
    theme_.trackBg = (trackBg == sf::Color::White ? theme_.trackBg : trackBg);
    theme_.handle =
        (handleColor == sf::Color::Cyan ? theme_.handle : handleColor);

    // track layers
    trackBg_.setFillColor(theme_.trackBg);
    trackFill_.setFillColor(theme_.trackFill);

    // handle
    handle_.setFillColor(theme_.handle);

    setHandleRadiusFromHeight();
    syncTrack();

    val_ = (range_.x + range_.y) * 0.5f;
    syncHandleFromValue();

    // text
    text_.setFont(font_);
    text_.setCharacterSize(height_ * 1.6f);

    oss << std::fixed << std::setprecision(2);
    oss << name_ + ": " << val_;
    text_.setString(oss.str());
  }

  void setPosition(sf::Vector2f pos) {
    position_ = pos;
    text_.setPosition({pos.x, pos.y + height_ * 1.5f});
    syncTrack();
    syncHandleFromValue();
  }

  void setSize(sf::Vector2f size) {
    width_ = size.x;
    height_ = size.y;
    syncTrack();
    setHandleRadiusFromHeight();
    syncHandleFromValue();
  }

  sf::Vector2f getSize() const noexcept { return {width_, height_}; };

  bool contains(sf::Vector2f pos) {
    return handle_.getGlobalBounds().contains(static_cast<sf::Vector2f>(pos));
  }

  void move(sf::Vector2f pos) {
    float x = static_cast<float>(pos.x);
    x = std::clamp(x, position_.x, position_.x + width_);

    val_ = clampToRange(xToValue(x));
    syncHandleFromValue();
    updateText();
  }

  void setActive(bool active) {
    isDragging = active;
    if (isDragging) {
      // apply dark overlay by reducing brightness
      handle_.setFillColor(sf::Color(
          static_cast<uint8_t>(theme_.handle.r * 0.7f),
          static_cast<uint8_t>(theme_.handle.g * 0.7f),
          static_cast<uint8_t>(theme_.handle.b * 0.7f), theme_.handle.a));
    } else {
      handle_.setFillColor(theme_.handle);
    }
  }

  void draw(sf::RenderTarget &rt) const {
    // draw order: track background -> active fill -> handle
    rt.draw(trackBg_);
    rt.draw(trackFill_);
    rt.draw(handle_);
    rt.draw(text_);
  }
};

#endif
