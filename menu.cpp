#include "menu.hpp"

#include "control-icons.hpp"

#include "engine/engine.hpp"

Menu::Menu(std::string_view title, std::vector<Item> items, const blit::Font &font) : blit::Menu(title, nullptr, 0, font), items_vec(std::move(items)) {
    this->items = items_vec.data();
    num_items = items_vec.size();

    item_h = font.char_h + 2;
    item_adjust_y = 0;

    header_h = item_h;
    footer_h = 0;
    margin_y = 0;

    background_colour = blit::Pen(0x11, 0x11, 0x11);
    foreground_colour = blit::Pen(0xF7, 0xF7, 0xF7);
    selected_item_background = blit::Pen(0x22, 0x22, 0x22);
}

void Menu::set_on_item_activated(void (*func)(const Item &)) {
    on_item_pressed = func;
}

void Menu::render_item(const Item &item, int y, int index) const {
    blit::Menu::render_item(item, y, index);

    if(index == current_item) {
        const int iconSize = font.char_h > 8 ? 12 : 8;
        blit::Point iconPos = blit::Point(display_rect.x + display_rect.w - item_padding_x -iconSize, y + 1); // from the top-right
        controlIcons.render(ControlIcons::Icon::A, iconPos, foreground_colour, iconSize);
    }
}

void Menu::item_activated(const Item &item) {
    if(on_item_pressed)
        on_item_pressed(item);
}