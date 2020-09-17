#include "menu.hpp"

#include "control-icons.hpp"

#include "engine/api.hpp"
#include "engine/engine.hpp"
#include "engine/input.hpp"

Menu::Menu(std::string_view title, std::vector<Item> items, const blit::Font &font) : title(title), items(std::move(items)), font(font)
{
}

void Menu::render()
{
    const int itemHeight = font.char_h + 2;
    const int iconSize = font.char_h > 8 ? 12 : 8;

    // title
    blit::Rect r(displayRect.tl(), blit::Size(displayRect.w, itemHeight));
    blit::Rect clipped = r;

    blit::screen.pen = blit::Pen(0xD7, 0xD7, 0xD7);
    blit::screen.rectangle(r);
    blit::screen.pen = blit::Pen(0x22, 0x22, 0x22);

    r.x += itemPadding;
    r.w -= itemPadding * 2;
    blit::screen.clip = clipped;
    blit::screen.text(std::string(title), font, r, true, blit::TextAlign::center_left);

    // items list
    // reserve space to display title
    auto itemsDisplayRect = displayRect;
    itemsDisplayRect.h -= itemHeight;
    itemsDisplayRect.y += itemHeight;

    int y = 0;
    int i = 0;

    // scrolling
    int totalHeight = items.size() * itemHeight;
    int selectedY = selectedItem * itemHeight;
    int yOff = itemsDisplayRect.h / 2 - selectedY;

    if(yOff < -(totalHeight - itemsDisplayRect.h))
        yOff = -(totalHeight - itemsDisplayRect.h);

    yOff = std::min(0, yOff);

    for(auto &f : items)
    {
        if(y + yOff > itemsDisplayRect.h)
            break;

        if(y + yOff + itemHeight < 0)
        {
            i++;
            y += itemHeight;
            continue;
        }

        // background
        if(i == selectedItem)
            blit::screen.pen = blit::Pen(0xF7, 0xF7, 0xF7);
        else if(i % 2)
            blit::screen.pen = blit::Pen(0x22, 0x22, 0x22);
        else
            blit::screen.pen = blit::Pen(0x11, 0x11, 0x11);

        blit::Rect r(itemsDisplayRect.x, itemsDisplayRect.y + yOff + y, itemsDisplayRect.w, itemHeight);
        auto &clipped = blit::screen.clip;
        clipped = r.intersection(itemsDisplayRect);

        blit::screen.rectangle(clipped);

        // secondary text (right before the button icon)
        auto secondaryText = "";
        const int32_t secTextWidth = blit::screen.measure_text(secondaryText, font).w;
        const int rightIconSpaceW = secTextWidth + iconSize + itemPadding + 4;

        if(i == selectedItem)
        {
            clipped.w -= rightIconSpaceW; // clip out the icon/ secondary text
            blit::screen.pen = blit::Pen(0x22, 0x22, 0x22);
        }
        else
            blit::screen.pen = blit::Pen(0xF7, 0xF7, 0xF7);

        r.h += font.spacing_y; // account for vertical spacing in alignment

        r.x += itemPadding;
        r.w -= itemPadding * 2;

        blit::screen.text(std::string(f.text), font, r, true, blit::TextAlign::center_left);

        // right icon / secondary text
        if(i == selectedItem)
        {
            clipped.w += rightIconSpaceW;
            blit::Point iconPos = r.tr() + blit::Point(-iconSize, (itemHeight - font.char_h) / 2); // from the top-right
            r.w -= iconSize + 2;

            blit::screen.text(secondaryText, font, r, true, blit::TextAlign::center_right);
            controlIcons.render(ControlIcons::Icon::A, iconPos, blit::Pen(0x22, 0x22, 0x22), iconSize);
        }

        y += itemHeight;
        i++;
    }

    blit::screen.clip = blit::Rect(blit::Point(0), blit::screen.bounds);
}

void Menu::update(uint32_t time)
{
    unsigned int upDown = blit::Button::DPAD_UP | blit::Button::DPAD_DOWN;

    if(blit::buttons.pressed & upDown)
        repeatStartTime = time;

    // repeat delay
    if((time - repeatStartTime) % 200 == 0)
    {
        if(blit::buttons & blit::Button::DPAD_UP)
            selectedItem--;
        else if(blit::buttons & blit::Button::DPAD_DOWN)
            selectedItem++;

        if(selectedItem == -1)
            selectedItem = items.size() - 1;
        else if(selectedItem >= static_cast<int>(items.size()))
            selectedItem = 0;
    }

    // A released
    if(blit::buttons.released & blit::Button::A)
    {
        if(onItemPressed)
            onItemPressed(items[selectedItem], selectedItem);
    }
    // B released
    else if(blit::buttons.released & blit::Button::B)
    {
    }
}

void Menu::setDisplayRect(blit::Rect r)
{
    displayRect = r;
}

void Menu::setOnItemPressed(void (*func)(Item &, int))
{
    onItemPressed = func;
}