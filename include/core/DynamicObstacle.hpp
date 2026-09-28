// DynamicObstacle.hpp - Moving obstacle that bounces within a corridor.
#pragma once

#include <utility>
#include <cstdint>

namespace warehouse {

enum class ObstacleAxis : uint8_t { HORIZONTAL, VERTICAL };

/// A dynamic obstacle that bounces linearly within [minPos, maxPos].
class DynamicObstacle {
public:
    const int id;

    DynamicObstacle(int id,
                    int col, int row,
                    ObstacleAxis axis,
                    int minPos, int maxPos,
                    float speed    = 2.0f,
                    int   cellSize = 32);

    // ── Current position ──────────────────────────────────────────────────
    [[nodiscard]] int col() const noexcept { return m_col; }
    [[nodiscard]] int row() const noexcept { return m_row; }
    [[nodiscard]] std::pair<int,int> pos() const noexcept { return {m_col, m_row}; }

    // ── Pixel position for smooth rendering ───────────────────────────────
    [[nodiscard]] float pixelX() const noexcept { return m_pixelX; }
    [[nodiscard]] float pixelY() const noexcept { return m_pixelY; }

    // ── Direction for rendering arrow ─────────────────────────────────────
    [[nodiscard]] int direction() const noexcept { return m_direction; }
    [[nodiscard]] ObstacleAxis axis() const noexcept { return m_axis; }

    /// Advance the obstacle. Returns true if grid cell changed.
    bool update(float dt, float speedMultiplier = 1.0f);

    /// Reset to given position.
    void reset(int col, int row);

    // ── Speed ─────────────────────────────────────────────────────────────
    void setSpeed(float s) noexcept { m_speed = s; }
    [[nodiscard]] float speed() const noexcept { return m_speed; }

    void setCellSize(int cs) noexcept { m_cellSize = cs; }

private:
    int   m_col, m_row;
    float m_pixelX, m_pixelY;
    float m_progress{0.0f};
    int   m_direction{1};     ///< +1 or -1

    ObstacleAxis m_axis;
    int   m_minPos, m_maxPos;
    float m_speed;
    int   m_cellSize;

    int   m_initCol, m_initRow;  ///< Saved for reset()

    [[nodiscard]] float centreX(int c) const noexcept;
    [[nodiscard]] float centreY(int r) const noexcept;
    void updatePixels();
};

}  // namespace warehouse
