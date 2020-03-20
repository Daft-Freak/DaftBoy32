#pragma once

#include <cstdint>
#include <set>
#include <string>
#include <vector>

#include "engine/file.hpp"
#include "graphics/font.hpp"
#include "types/rect.hpp"

class FileBrowser final
{
public:
    FileBrowser(const blit::Font &font = blit::minimal_font);

    void init();

    void render();
    void update(uint32_t time);

    void setDisplayRect(blit::Rect r);
    void setExtensions(std::set<std::string> exts);

    void setOnFileOpen(void (*func)(std::string));

private:
    void updateList();

    const blit::Font &font;

    static const int itemPadding = 2; // x padding

    std::vector<blit::FileInfo> files;
    int selectedFile = 0;
    std::string curDir;

    uint32_t lastButtonState = 0;
    uint32_t repeatStartTime = 0;

    blit::Rect displayRect;
    std::set<std::string> fileExts;
    void (*onFileOpen)(std::string) = nullptr;
};