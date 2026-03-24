/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */
/*
  OSD backend for SITL. Uses SFML media library. See
  https://www.sfml-dev.org/index.php

  To use install SFML libraries, and run sim_vehicle.py with --osd
  option. Then set OSD_TYPE to 2
 */
#ifdef WITH_SITL_OSD

#include "AP_OSD_SITL.h"
#include <AP_HAL/Util.h>
#include <AP_HAL/Semaphores.h>
#include <AP_HAL/Scheduler.h>
#include <AP_ROMFS/AP_ROMFS.h>
#include <SITL/SITL.h>
#include <utility>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include "pthread.h"

#include <AP_Notify/AP_Notify.h>

extern const AP_HAL::HAL &hal;

/*
  load *.bin font file, in MAX7456 format
 */
void AP_OSD_SITL::load_font(void)
{
    last_font = get_font_num();
    FileData *fd = load_font_data(last_font);
    if (fd == nullptr || fd->length != 54 * 256) {
        AP_HAL::panic("Bad font file");
    }
    for (uint16_t i=0; i<256; i++) {
        const uint8_t *c = &fd->data[i*54];
        // each pixel is 4 bytes, RGBA
        sf::Uint8 *pixels = NEW_NOTHROW sf::Uint8[char_width * char_height * 4];
        if (!font[i].create(char_width, char_height)) {
            AP_HAL::panic("Failed to create texture");
        }
        for (uint16_t y=0; y<char_height; y++) {
            for (uint16_t x=0; x<char_width; x++) {
                // 2 bits per pixel
                uint16_t bitoffset = (y*char_width+x)*2;
                uint8_t byteoffset = bitoffset / 8;
                uint8_t bitshift = 6-(bitoffset % 8);
                uint8_t v = (c[byteoffset] >> bitshift) & 3;
                sf::Uint8 *p = &pixels[(y*char_width+x)*4];
                switch (v) {
                case 0:
                    p[0] = 0;
                    p[1] = 0;
                    p[2] = 0;
                    p[3] = 255;
                    break;
                case 1:
                case 3:
                    p[0] = 0;
                    p[1] = 0;
                    p[2] = 0;
                    p[3] = 0;
                    break;
                case 2:
                    p[0] = 255;
                    p[1] = 255;
                    p[2] = 255;
                    p[3] = 255;
                    break;
                }

            }
        }
        font[i].update(pixels);
    }
    delete fd;
}

void AP_OSD_SITL::write(uint8_t x, uint8_t y, const char* text)
{
    if (y >= video_lines || text == nullptr) {
        return;
    }
    WITH_SEMAPHORE(mutex);

    while ((x < video_cols) && (*text != 0)) {
        getbuffer(buffer, y, x) = *text;
        ++text;
        ++x;
    }
}

void AP_OSD_SITL::clear(void)
{
    AP_OSD_Backend::clear();
    WITH_SEMAPHORE(mutex);
    memset(buffer, 0, video_cols*video_lines);
}

void AP_OSD_SITL::flush(void)
{
    counter++;
}

void AP_OSD_SITL::draw_crosshair(sf::RenderWindow *window)
{
    const auto *sitl = AP::sitl();
    if (!sitl)
        return; 
    
    double crosshair_x = sitl->state.crosshair_x;
    double crosshair_y = sitl->state.crosshair_y;
    
    if (fabsf(crosshair_x) < 0.001f && fabsf(crosshair_y) < 0.001f) 
    {
        crosshair_x = 0.5f;
        crosshair_y = 0.5f;
    }
    
    sf::Vector2u window_size = window->getSize();
    float cx = window_size.x * crosshair_x;
    float cy = window_size.y * crosshair_y;
    
    // Размер перекрестия относительно размера окна
    float len = std::min(window_size.x, window_size.y) / 25.0f; 
    float gap = len / 4.0f;
    
    sf::Color col(255, 255, 255, 200);

    sf::VertexArray lines(sf::Lines, 8);
    // горизонталь левая
    lines[0].position = {cx - gap - len, cy};  lines[0].color = col;
    lines[1].position = {cx - gap,        cy};  lines[1].color = col;
    // горизонталь правая
    lines[2].position = {cx + gap,         cy};  lines[2].color = col;
    lines[3].position = {cx + gap + len,   cy};  lines[3].color = col;
    // вертикаль верхняя
    lines[4].position = {cx, cy - gap - len};  lines[4].color = col;
    lines[5].position = {cx, cy - gap       };  lines[5].color = col;
    // вертикаль нижняя
    lines[6].position = {cx, cy + gap       };  lines[6].color = col;
    lines[7].position = {cx, cy + gap + len };  lines[7].color = col;
    
    window->draw(lines);
}

void AP_OSD_SITL::draw_timestamp(sf::RenderWindow *window, sf::Font &ts_font)
{
    char buf[64];

    const auto *sitl = AP::sitl();
    if (!sitl) 
        return;
    
    uint64_t last_osd_timestamp = sitl->state.timestamp_ms;
    
    if (last_osd_timestamp != 0) 
    {
        time_t seconds = last_osd_timestamp / 1000;
        uint32_t milliseconds = last_osd_timestamp % 1000;
        
        struct tm* tm_info = localtime(&seconds);
        
        // Формат: ГГГГ-ММ-ДД ЧЧ:ММ:СС.мсс
        snprintf(buf, sizeof(buf), "%04d-%02d-%02d %02d:%02d:%02d.%03u",
                 tm_info->tm_year + 1900, 
                 tm_info->tm_mon + 1,       
                 tm_info->tm_mday,       
                 tm_info->tm_hour,        
                 tm_info->tm_min,         
                 tm_info->tm_sec,           
                 milliseconds);         
    } else {
        snprintf(buf, sizeof(buf), "----:--:-- --:--:--.---");
    }

    sf::Text text;
    text.setFont(ts_font);
    text.setString(buf);
    text.setCharacterSize(12); 
    text.setFillColor(sf::Color(255, 255, 255, 220));
    text.setOutlineColor(sf::Color(0, 0, 0, 180));
    text.setOutlineThickness(1.0f);
    text.setPosition(6.0f, 4.0f);
    window->draw(text);
}

// main loop of graphics thread
void AP_OSD_SITL::update_thread(void)
{
    load_font();
    
    const char* font_paths[] = {
        "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf",
        "/usr/share/fonts/truetype/liberation/LiberationMono-Regular.ttf",
        "/usr/share/fonts/truetype/freefont/FreeMono.ttf",
        nullptr
    };
    for (int i = 0; font_paths[i]; i++) {
        if (_ts_font.loadFromFile(font_paths[i])) {
            _ts_font_loaded = true;
            break;
        }
    }
    
    {
        WITH_SEMAPHORE(AP::notify().sf_window_mutex);
        w = NEW_NOTHROW sf::RenderWindow(sf::VideoMode(video_cols*(char_width+char_spacing)*char_scale,
                                               video_lines*(char_height+char_spacing)*char_scale),
                                 "OSD");
    }
    if (!w) {
        AP_HAL::panic("Unable to create OSD window");
    }

    while (true) {
        {
            WITH_SEMAPHORE(AP::notify().sf_window_mutex);
            sf::Event event;
            while (w->pollEvent(event)) {
                if (event.type == sf::Event::Closed) {
                    w->close();
                }
            }
            if (!w->isOpen()) {
                break;
            }
            if (counter != last_counter) {
                last_counter = counter;

                uint8_t buffer2[video_lines][video_cols];
                {
                    WITH_SEMAPHORE(mutex);
                    memcpy(buffer2, buffer, sizeof(buffer2));
                }
                w->clear();

                for (uint8_t y=0; y<video_lines; y++) {
                    for (uint8_t x=0; x<video_cols; x++) {
                        uint16_t px = x * (char_width+char_spacing) * char_scale;
                        uint16_t py = y * (char_height+char_spacing) * char_scale;
                        sf::Sprite s;
                        uint8_t c = buffer2[y][x];
                        s.setTexture(font[c]);
                        s.setPosition(sf::Vector2f(px, py));
                        s.scale(sf::Vector2f(char_scale,char_scale));
                        w->draw(s);
                    }
                }

                w->display();
                draw_crosshair(w);
                if (_ts_font_loaded) {
                    draw_timestamp(w, _ts_font);
                }
                w->display();  // second display after overlay
                
                if (last_font != get_font_num()) {
                    load_font();
                }
            }
        }
        usleep(10000);
    }
}

// trampoline for update thread
void *AP_OSD_SITL::update_thread_start(void *obj)
{
    ((AP_OSD_SITL *)obj)->update_thread();
    return nullptr;
}

// initialise backend
bool AP_OSD_SITL::init(void)
{    
    pthread_create(&thread, NULL, update_thread_start, this);
    return true;
}

AP_OSD_Backend *AP_OSD_SITL::probe(AP_OSD &osd)
{
    AP_OSD_SITL *backend = NEW_NOTHROW AP_OSD_SITL(osd);
    if (!backend) {
        return nullptr;
    }
    if (!backend->init()) {
        delete backend;
        return nullptr;
    }
    return backend;
}

AP_OSD_SITL::AP_OSD_SITL(AP_OSD &osd):
    AP_OSD_Backend(osd)
{
    const auto *_sitl = AP::sitl();
    video_lines = _sitl->osd_rows;
    video_cols = _sitl->osd_columns;
    buffer = (uint8_t *)malloc(video_lines*video_cols);
}

#endif // WITH_SITL_OSD