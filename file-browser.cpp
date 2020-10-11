#include "file-browser.hpp"

#include "control-icons.hpp"

#include "engine/engine.hpp"

FileBrowser::FileBrowser(const blit::Font &font) : Menu("", nullptr, 0, font) {
    // too early
    //files = blit::list_files("");

    item_h = font.char_h + 2;
    item_adjust_y = 0;

    header_h = item_h;
    footer_h = 0;
    margin_y = 0;

    background_colour = blit::Pen(0x11, 0x11, 0x11);
    foreground_colour = blit::Pen(0xF7, 0xF7, 0xF7);
    selected_item_background = blit::Pen(0x22, 0x22, 0x22);
}

void FileBrowser::init() {
    update_list();
}

void FileBrowser::render()
{
    blit::Menu::render();

    const int iconSize = font.char_h > 8 ? 12 : 8;

    const int32_t backTextWidth = blit::screen.measure_text("Back", font).w;

    blit::Rect r(display_rect.tl(), blit::Size(display_rect.w, header_h));

    blit::screen.pen = header_foreground;

    r.x += item_padding_x;
    r.w -= item_padding_x * 2;

    // back icon
    if(!cur_dir.empty()) {
        blit::Point iconOffset(-(backTextWidth + iconSize + 2), 1); // from the top-right

        blit::screen.text("Back", font, r, true, blit::TextAlign::center_right);
        controlIcons.render(ControlIcons::Icon::B, r.tr() + iconOffset, header_foreground, iconSize);
    }
}

void FileBrowser::set_extensions(std::set<std::string> exts) {
    file_exts = exts;
}

void FileBrowser::set_on_file_open(void (*func)(std::string)) {
    on_file_open = func;
}

void FileBrowser::update_list() {
    title = cur_dir;

    files = blit::list_files(cur_dir.substr(0, cur_dir.length() - 1));

    std::sort(files.begin(), files.end(), [](blit::FileInfo &a, blit::FileInfo & b){return a.name < b.name;});

    if(file_exts.empty())
        return;

    // filter by extensions
    files.erase(std::remove_if(files.begin(), files.end(), [this](const blit::FileInfo &f) {
        if(!(f.flags & blit::FileFlags::directory)) {
            std::string ext;
            auto dotPos = f.name.find_last_of('.');
            if(dotPos != std::string::npos)
                ext = f.name.substr(dotPos);

            // convert to lower case
            std::for_each(ext.begin(), ext.end(), [](char & c) {c = tolower(c);});

            if(file_exts.find(ext) == file_exts.end())
                return true;
        }

        return false;
    }), files.end());

    // update menu items
    file_items.resize(files.size());

    unsigned int i = 0;
    for(auto &file : files) {
        if(file.flags & blit::FileFlags::directory)
            file.name += "/";

        file_items[i].id = i;
        file_items[i++].label = file.name.c_str();
    }

    set_items(file_items.data(), file_items.size());
}

void FileBrowser::render_item(const Item &item, int y, int index) const {
    blit::Menu::render_item(item, y, index);

    if(index == current_item) {
        const int iconSize = font.char_h > 8 ? 12 : 8;

        blit::Rect r(display_rect.x + item_padding_x, y, display_rect.w - item_padding_x * 2 - iconSize - 2, item_h);
        blit::Point iconPos = blit::Point(display_rect.x + display_rect.w - item_padding_x -iconSize, y + 1); // from the top-right
        controlIcons.render(ControlIcons::Icon::A, iconPos, foreground_colour, iconSize);
    }
}

void FileBrowser::update_item(const Item &item) {
    if(blit::buttons.released & blit::Button::B) {
        if(!cur_dir.empty()) {
            // go up
            auto pos = cur_dir.find_last_of('/', cur_dir.length() - 2);
            if(pos == std::string::npos)
                cur_dir = "";
            else
                cur_dir = cur_dir.substr(0, pos + 1);

            update_list();
        }
    }
}

void FileBrowser::item_activated(const Item &item){
    if(files[current_item].flags & blit::FileFlags::directory) {
        cur_dir += files[current_item].name;
        update_list();
    }
    else if(on_file_open)
        on_file_open(cur_dir + files[current_item].name);
}