#pragma once

#include <cstdint>
#include <vector>

#include "engine/menu.hpp"

class Menu final : public blit::Menu {
public:
    Menu(std::string_view title, std::vector<Item> items, const blit::Font &font = blit::minimal_font);

    void set_on_item_activated(void (*func)(const Item &));

private:
    void render_item(const Item &item, int y, int index) const override;
    void item_activated(const Item &item) override;

    std::vector<Item> items_vec;

    void (*on_item_pressed)(const Item &) = nullptr;
};