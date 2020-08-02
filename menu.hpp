#pragma once

#include <cstdint>
#include <string_view>
#include <vector>

#include "graphics/font.hpp"
#include "types/rect.hpp"

class Menu final
{
public:
    class Item
    {
    public:
        std::string_view text;
        int id = -1;
    };

    Menu(std::string_view title, std::vector<Item> items, const blit::Font &font = blit::minimal_font);

    void render();
    void update(uint32_t time);

    void setDisplayRect(blit::Rect r);

    void setOnItemPressed(void (*func)(Item &, int));

private:

    static const int itemPadding = 2; // x padding

    std::string_view title;

    std::vector<Item> items;
    int selectedItem = 0;

    uint32_t repeatStartTime = 0;

    const blit::Font &font;
    blit::Rect displayRect;
    void (*onItemPressed)(Item &, int) = nullptr;
};