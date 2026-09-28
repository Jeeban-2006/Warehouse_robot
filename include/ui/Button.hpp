// Button.hpp - SDL2 button widget.
#pragma once
#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>
#include <string>
#include <functional>
#include "Renderer.hpp"

namespace warehouse {

class Button {
public:
    Button(int x, int y, int w, int h,
           std::string label,
           std::function<void()> onClick,
           Color color = Colors::BTN_NORMAL);

    void setLabel(const std::string& l) { m_label = l; }
    void setEnabled(bool e)  noexcept { m_enabled = e; }
    void setToggled(bool t)  noexcept { m_toggled = t; }
    [[nodiscard]] bool enabled()  const noexcept { return m_enabled; }
    [[nodiscard]] bool toggled()  const noexcept { return m_toggled; }
    [[nodiscard]] bool contains(int mx, int my) const noexcept;

    void handleEvent(const SDL_Event& e);
    void draw(SDL_Renderer* renderer, TTF_Font* font) const;

private:
    SDL_Rect m_rect;
    std::string m_label;
    std::function<void()> m_onClick;
    Color m_color;
    bool  m_enabled{true};
    bool  m_hovered{false};
    bool  m_toggled{false};
};

}  // namespace warehouse
