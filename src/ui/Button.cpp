// Button.cpp - SDL2 button widget implementation.
#include "ui/Button.hpp"
#include <algorithm>

namespace warehouse {

Button::Button(int x, int y, int w, int h,
               std::string label,
               std::function<void()> onClick,
               Color color)
    : m_rect{x, y, w, h},
      m_label(std::move(label)),
      m_onClick(std::move(onClick)),
      m_color(color)
{}

bool Button::contains(int mx, int my) const noexcept {
    return mx >= m_rect.x && mx < m_rect.x + m_rect.w &&
           my >= m_rect.y && my < m_rect.y + m_rect.h;
}

void Button::handleEvent(const SDL_Event& e) {
    if (!m_enabled) return;
    if (e.type == SDL_MOUSEMOTION) {
        m_hovered = contains(e.motion.x, e.motion.y);
    } else if (e.type == SDL_MOUSEBUTTONDOWN && e.button.button == SDL_BUTTON_LEFT) {
        if (contains(e.button.x, e.button.y) && m_onClick)
            m_onClick();
    }
}

void Button::draw(SDL_Renderer* renderer, TTF_Font* font) const {
    Color bg = m_enabled
               ? (m_toggled ? Colors::BTN_ACTIVE
                             : (m_hovered ? Colors::BTN_HOVER : m_color))
               : Colors::BTN_DISABLED;

    SDL_SetRenderDrawColor(renderer, bg.r, bg.g, bg.b, bg.a);
    SDL_RenderFillRect(renderer, &m_rect);

    // Border
    Color border = m_enabled
                   ? (m_toggled ? Colors::ACCENT : Color{60,65,90})
                   : Color{38,40,60};
    SDL_SetRenderDrawColor(renderer, border.r, border.g, border.b, border.a);
    SDL_RenderDrawRect(renderer, &m_rect);

    // Label
    if (!font || m_label.empty()) return;
    int tw = 0, th = 0;
    TTF_SizeText(font, m_label.c_str(), &tw, &th);
    int tx = m_rect.x + (m_rect.w - tw)/2;
    int ty = m_rect.y + (m_rect.h - th)/2;

    Color tc = m_enabled ? Colors::TEXT_BRIGHT : Colors::TEXT_DIM;
    SDL_Surface* surf = TTF_RenderText_Blended(font, m_label.c_str(), tc.sdl());
    if (!surf) return;
    SDL_Texture* tex = SDL_CreateTextureFromSurface(renderer, surf);
    if (tex) {
        SDL_Rect dst{tx, ty, surf->w, surf->h};
        SDL_RenderCopy(renderer, tex, nullptr, &dst);
        SDL_DestroyTexture(tex);
    }
    SDL_FreeSurface(surf);
}

}  // namespace warehouse
