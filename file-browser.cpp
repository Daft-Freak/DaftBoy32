#include "file-browser.hpp"

#include "control-icons.hpp"

#include "engine/api.hpp"
#include "engine/engine.hpp"
#include "engine/input.hpp"

FileBrowser::FileBrowser(const blit::Font &font) : font(font)
{
    // too early
    //files = blit::list_files("");
}

void FileBrowser::init()
{
    updateList();

    // default
    if(displayRect.w == 0 && displayRect.h == 0)
        displayRect = blit::Rect(blit::Point(0, 0), blit::screen.bounds);
}

void FileBrowser::render()
{
    const int itemHeight = font.char_h + 2;
    const int iconSize = font.char_h > 8 ? 12 : 8;

    const int32_t openTextWidth = blit::screen.measure_text("Open", font).w;
    const int openIconSpaceW = openTextWidth + iconSize + itemPadding + 4;

    const int32_t backTextWidth = blit::screen.measure_text("Back", font).w;
    const int backIconSpaceW = backTextWidth + iconSize + itemPadding + 4;

    // current dir info
    blit::Rect r(displayRect.tl(), blit::Size(displayRect.w, itemHeight));
    blit::Rect clipped = r;
    clipped.w -= backIconSpaceW;
    blit::screen.pen = blit::Pen(0xD7, 0xD7, 0xD7);
    blit::screen.rectangle(r);
    blit::screen.pen = blit::Pen(0x22, 0x22, 0x22);

    r.x += itemPadding;
    r.w -= itemPadding * 2;
    blit::screen.text(curDir + "/", font, r, true, blit::TextAlign::center_left, clipped);

    // back icon
    if(!curDir.empty())
    {
        blit::Point iconOffset(-(backTextWidth + iconSize + 2), (itemHeight - font.char_h) / 2); // from the top-right

        blit::screen.text("Back", font, r, true, blit::TextAlign::center_right);
        controlIcons.render(ControlIcons::Icon::B, r.tr() + iconOffset, blit::Pen(0x22, 0x22, 0x22), iconSize);
    }

    // files list
    // reserve space to display current dir
    auto filesDisplayRect = displayRect;
    filesDisplayRect.h -= itemHeight;
    filesDisplayRect.y += itemHeight;

    int y = 0;
    int i = 0;

    // scrolling
    int totalHeight = files.size() * itemHeight;
    int selectedY = selectedFile * itemHeight;
    int yOff = filesDisplayRect.h / 2 - selectedY;

    if(yOff < -(totalHeight - filesDisplayRect.h))
        yOff = -(totalHeight - filesDisplayRect.h);

    yOff = std::min(0, yOff);

    for(auto &f : files)
    {
        if(y + yOff > filesDisplayRect.h)
            break;

        if(y + yOff + itemHeight < 0)
        {
            i++;
            y += itemHeight;
            continue;
        }

        // background
        if(i == selectedFile)
            blit::screen.pen = blit::Pen(0xF7, 0xF7, 0xF7);
        else if(i % 2)
            blit::screen.pen = blit::Pen(0x22, 0x22, 0x22);
        else
            blit::screen.pen = blit::Pen(0x11, 0x11, 0x11);

        auto str = f.name + ((f.flags & blit::FileFlags::directory) ? "/" : "");

        blit::Rect r(filesDisplayRect.x, filesDisplayRect.y + yOff + y, filesDisplayRect.w, itemHeight);
        blit::Rect clipped = r.intersection(filesDisplayRect);

        blit::screen.rectangle(clipped);

        if(i == selectedFile)
        {
            clipped.w -= openIconSpaceW; // clip out the open icon
            blit::screen.pen = blit::Pen(0x22, 0x22, 0x22);
        }
        else
            blit::screen.pen = blit::Pen(0xF7, 0xF7, 0xF7);

        r.h += font.spacing_y; // account for vertical spacing in alignment

        r.x += itemPadding;
        r.w -= itemPadding * 2;

        blit::screen.text(str, font, r, true, blit::TextAlign::center_left, clipped);

        // open icon
        if(i == selectedFile)
        {
            clipped.w += openIconSpaceW;
            blit::Point iconOffset(-(openTextWidth + iconSize + 2), (itemHeight - font.char_h) / 2); // from the top-right

            blit::screen.text("Open", font, r, true, blit::TextAlign::center_right, clipped);
            controlIcons.render(ControlIcons::Icon::A, r.tr() + iconOffset, blit::Pen(0x22, 0x22, 0x22), iconSize);
        }

        y += itemHeight;
        i++;
    }
}

void FileBrowser::update(uint32_t time)
{
    unsigned int upDown = blit::Button::DPAD_UP | blit::Button::DPAD_DOWN;

    if((blit::buttons & upDown) != (lastButtonState & upDown))
        repeatStartTime = time;

    // repeat delay
    if((time - repeatStartTime) % 200 == 0)
    {
        if(blit::buttons & blit::Button::DPAD_UP)
            selectedFile--;
        else if(blit::buttons & blit::Button::DPAD_DOWN)
            selectedFile++;

        if(selectedFile == -1)
            selectedFile = files.size() - 1;
        else if(selectedFile >= static_cast<int>(files.size()))
            selectedFile = 0;
    }

    // A released
    if((lastButtonState & blit::Button::A) && !(blit::buttons & blit::Button::A))
    {
        if(files[selectedFile].flags & blit::FileFlags::directory)
        {
            if(!curDir.empty())
                curDir += "/";

            curDir += files[selectedFile].name;
            updateList();
        }
        else if(onFileOpen)
            onFileOpen(curDir + (curDir.empty() ? "" : "/") + files[selectedFile].name);
    }
    // B released
    else if((lastButtonState & blit::Button::B) && !(blit::buttons & blit::Button::B))
    {
        if(!curDir.empty())
        {
            // go up
            auto pos = curDir.find_last_of('/');
            if(pos == std::string::npos)
                curDir = "";
            else
                curDir = curDir.substr(0, pos);

            updateList();
        }
    }

    lastButtonState = blit::buttons;
}

void FileBrowser::setDisplayRect(blit::Rect r)
{
    displayRect = r;
}

void FileBrowser::setExtensions(std::set<std::string> exts)
{
    fileExts = exts;
}

void FileBrowser::setOnFileOpen(void (*func)(std::string))
{
    onFileOpen = func;
}

void FileBrowser::updateList()
{
    files = blit::list_files(curDir);
    selectedFile = 0;

    std::sort(files.begin(), files.end(), [](blit::FileInfo &a, blit::FileInfo & b){return a.name < b.name;});

    if(fileExts.empty())
        return;

    // filter by extensions
    files.erase(std::remove_if(files.begin(), files.end(), [this](const blit::FileInfo &f)
    {
        if(!(f.flags & blit::FileFlags::directory))
        {
            std::string ext;
            auto dotPos = f.name.find_last_of('.');
            if(dotPos != std::string::npos)
                ext = f.name.substr(dotPos);

            // convert to lower case
            std::for_each(ext.begin(), ext.end(), [](char & c) {c = tolower(c);});

            if(fileExts.find(ext) == fileExts.end())
                return true;
        }

        return false;
    }), files.end());
}
