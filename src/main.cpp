#include "Arduino.h"
#define CAMERA_MODEL_WROVER_KIT
#include "esp_camera.h"
#include "camera_pins.h"

#include <WiFi.h>
#include "timer/timer.h"
#include "logger/logger.h"

#include <algorithm>
#include <cmath>
#include <initializer_list>
#include <type_traits>
#include <vector>
#include <unordered_map>
#include <array>
#include <utility>
#include <tuple>
#include <string>
#include <type_traits>

#include "esp_log.h"

#define IMG_WIDTH 320
#define IMG_HEIGHT 240

#define MIN_RAD 10 //Inclusive
#define MAX_RAD 25 //Inclusive

#define IN_RANGE(x,min,max) (((x) < (max)) && ((x) > (min)))

static const char* CAM = "CAM", *FUNC_TIMING = "TIME", *DEBUG = "DEBUG", *ERROR = "ERROR";

static const char* ssid = "ESP32 Live Stream";
static const char* password = "esp32_314159";

static camera_config_t camera_config = {
    .pin_pwdn  = PWDN_GPIO_NUM,
    .pin_reset = RESET_GPIO_NUM,
    .pin_xclk = XCLK_GPIO_NUM,
    .pin_sccb_sda = SIOD_GPIO_NUM,
    .pin_sccb_scl = SIOC_GPIO_NUM,

    .pin_d7 = Y9_GPIO_NUM,
    .pin_d6 = Y8_GPIO_NUM,
    .pin_d5 = Y7_GPIO_NUM,
    .pin_d4 = Y6_GPIO_NUM,
    .pin_d3 = Y5_GPIO_NUM,
    .pin_d2 = Y4_GPIO_NUM,
    .pin_d1 = Y3_GPIO_NUM,
    .pin_d0 = Y2_GPIO_NUM,
    .pin_vsync = VSYNC_GPIO_NUM,
    .pin_href = HREF_GPIO_NUM,
    .pin_pclk = PCLK_GPIO_NUM,

    .xclk_freq_hz = 20000000, //EXPERIMENTAL: Set to 16MHz on ESP32-S2 or ESP32-S3 to enable EDMA mode
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    .pixel_format = PIXFORMAT_JPEG, //YUV422,GRAYSCALE,RGB565,JPEG
    .frame_size = FRAMESIZE_QVGA, //QQVGA-UXGA, For ESP32, do not use sizes above QVGA when not JPEG. 
                                  // The performance of the ESP32-S series has improved a lot, but JPEG mode always gives better frame rates.

    .jpeg_quality = 5, //0-63, for OV series camera sensors, lower number means higher quality
    .fb_count = 2, //When jpeg mode is used, if fb_count more than one, the driver will work in continuous mode.
    .fb_location = CAMERA_FB_IN_PSRAM,
    .grab_mode = CAMERA_GRAB_LATEST//. Sets when buffers should be filled CAMERA_GRAB_WHEN_EMPTY CAMERA_GRAB_LATEST
    // .sccb_i2c_port = -1
};


enum class Color {Red, Blue, Black, Brown, Other, _NUM_COLORS};
constexpr auto _NumColors = static_cast<std::underlying_type_t<Color>>(Color::_NUM_COLORS);
struct colorStringsStruct {
    static constexpr std::array<std::string_view, _NumColors> rawStrings{{
        "Red", "Blue", "Black", "Brown", "Other"
    }};
    std::string_view operator[](Color c) {
        return rawStrings[static_cast<std::underlying_type_t<Color>>(c)];
    }
};
struct colorStringsStruct colorStringMap;

using _RGBcolor = std::array<uint8_t, 3>;
using _ColorShades = std::array<_RGBcolor, 3>;

struct ColorShadesStruct {
    const std::array<_ColorShades, _NumColors> rawShades;
    constexpr const _ColorShades& operator[](Color c) const {
        return rawShades[static_cast<std::underlying_type_t<Color>>(c)];
    }
};

constexpr ColorShadesStruct colorShadesMap {{{
    {{ // Red
        {{255,0,0}},
        {{255,89,89}},
        {{255,179,179}}
    }},
    {{ // Blue
        {{0,0,255}},
        {{89,89,255}},
        {{179,179,255}}
    }},
    {{ // Black
        {{0,0,0}},
        {{89,89,89}},
        {{179,179,179}}
    }},
    {{ // Brown
        {{150,75,0}},
        {{187,138,89}},
        {{224,201,179}}
    }},
    {{ // Other
        {{0,150,0}},
        {{89,187,89}},
        {{179,224,179}}
    }}
    // Enum Value	Variant 1: Vibrant (Base)	Variant 2: Medium (35% Tint)	Variant 3: Light (70% Tint)
    // Red	(255, 0, 0)	(255, 89, 89)	(255, 179, 179)
    // Blue	(0, 0, 255)	(89, 89, 255)	(179, 179, 255)
    // Black	(0, 0, 0)	(89, 89, 89)	(179, 179, 179)
    // Brown	(150, 75, 0)	(187, 138, 89)	(224, 201, 179)
    // Other	(0, 150, 0)	(89, 187, 89)	(179, 224, 179)
}}};

std::unordered_map<Color, int> ballCounts;

struct ImgAreaOfInterest {
    int top;
    int bottom;
    int left;
    int right;
};


enum class Mode {ShowCameraFeed, ShowColors, ShowGradient, ShowHoughTransform, ShowCircles};

struct HSV {
    int h=0; //[0-360]: red(0,120), green(120-240), blue(240-360)
    int s=0; //[0-255]: saturation, vividness
    int v=0; //[0-255]: value, brightness
    HSV(int r, int g, int b) {
        v = max({r,g,b}); 
        int v_min = min({r,g,b});
        int delta = v-v_min;


        if (delta == 0) h=0;
        else if (v == r) {
            h = (g-b)*60/delta+60;
        }
        else if (v == g) {
            h = (b-r)*60/delta+180;
        }
        else if (v==b) {
            h = (r-g)*60/delta+300;
        }

        s = v == 0 ? 0 : delta*255/v;
    }
};

// TODO: ADD MICROS FUNCTIONALITY
struct MyFuncTimer {
    private:
    static inline uint32_t static_time=0;

    public:
    const char* name;
    uint32_t member_time;
    MyFuncTimer(const char* s = ""): name(s), member_time(millis()) {}
    ~MyFuncTimer() {ESP_LOGD(FUNC_TIMING, "\t\t\tFunction %s took %lums", name, millis()-member_time);}


    static void setTime(uint32_t* t = nullptr) {
        if (t) *t = millis();
        else static_time = millis();
    };

    static void printTimeDiff() {
        ESP_LOGD(FUNC_TIMING, "\tFunction took %lums", millis()-static_time);
    }
    static void printTimeDiff(const char* s, uint32_t t = 0) {
        if (t) ESP_LOGD(FUNC_TIMING, "\tFunction %s took %lums", s, millis()-t);
        else ESP_LOGD(FUNC_TIMING, "\tFunction %s took %lums", s, millis()-static_time);
    }

};

struct Coordinate {
    uint16_t x, y;
};

struct Pixel {
    uint16_t x=0,y=0,i=0;
};

struct Ball {
    uint16_t x=0,y=0,r=0,i=0;
    Color color;
    Ball(const Pixel& p, uint16_t rad):
        x(p.x), y(p.y), r(rad), i(p.i){}
};

constexpr camera_fb_t rgbImgTemplate =  {.buf=nullptr, .len=IMG_HEIGHT*IMG_WIDTH*3, .width=IMG_WIDTH, .height=IMG_HEIGHT, .format=PIXFORMAT_RGB888, .timestamp={0,0}},
                grayscaleImgTemplate =  {.buf=nullptr, .len=IMG_HEIGHT*IMG_WIDTH, .width=IMG_WIDTH, .height=IMG_HEIGHT, .format=PIXFORMAT_GRAYSCALE, .timestamp={0,0}},
                uint16ImgTemplate =     {.buf=nullptr, .len=IMG_HEIGHT*IMG_WIDTH*2, .width=IMG_WIDTH, .height=IMG_HEIGHT,.format=PIXFORMAT_GRAYSCALE, .timestamp={0,0}};

camera_fb_t rgb888Img = rgbImgTemplate;
camera_fb_t colorImg = rgbImgTemplate;
camera_fb_t grayImg = grayscaleImgTemplate;
camera_fb_t gradImg = grayscaleImgTemplate;
camera_fb_t uint16Img = uint16ImgTemplate;
camera_fb_t outputImg = rgbImgTemplate;
camera_fb_t jpgImg = {.buf=nullptr, .len=0, .width=IMG_WIDTH, .height=IMG_HEIGHT, .format=PIXFORMAT_JPEG, .timestamp={0,0}};

camera_fb_t hsvImg = rgbImgTemplate;
camera_fb_t circleImg = grayscaleImgTemplate;

camera_fb_t finalHoughImg = grayscaleImgTemplate;

int16_t **circleCoordinates;

Mode mode = Mode::ShowColors;

WiFiServer server(8888);
WiFiClient client;
bool clientConnected = false;


void init_camera(){
    while (esp_camera_init(&camera_config) != ESP_OK) {
        ESP_LOGE(CAM, "Camera Failed");
        delay(5000);
    }
}

camera_fb_t* camera_capture(){
    camera_fb_t * fb = esp_camera_fb_get();
  
    while (!fb) {
        ESP_LOGE(CAM, "Camera Capture Failed");
        delay(5000);
        fb = esp_camera_fb_get();
    }
    ESP_LOGI(CAM, "Captured frame");
    return fb;
}

void logErrorAndRestart(const char* s) {
    ESP_LOGE(ERROR, "%s", s); 
    vTaskDelay(pdMS_TO_TICKS(10000));
    ESP.restart();
}

void allocate_camera_fb(camera_fb_t* fb) {
    fb->buf = (uint8_t*) ps_malloc(fb->len);
        if (fb->buf == NULL) {
            ESP_LOGE(CAM, "Failed to allocate buffer");
            delay(10000);
            ESP.restart();
        }
        memset(fb->buf, 0, fb->len);
}

void allocate_camera_fbs(std::initializer_list<camera_fb_t*> fbs) {
    for (int i = 0; camera_fb_t *fb : fbs) {
        allocate_camera_fb(fb);
        i++;
    }
}

void updateRGBBuffer(camera_fb_t *fb) {
    memset(rgb888Img.buf, 0, rgb888Img.len);
    while (!fmt2rgb888(fb->buf, fb->len, fb->format, rgb888Img.buf)) {
        Serial.println("Formatting Failed");
        delay(2000);
    }
  
}

void swapRBchannels(camera_fb_t *fb) {
    if (fb->format != PIXFORMAT_RGB888) {
        ESP_LOGE(CAM, "Frame buffer passed to swapRBchannels is not RGB formatted");
        delay(10000);
        ESP.restart();
    }
    uint8_t temp;
    for (int i = 0; i < fb->len; i+=3) {
        temp = fb->buf[i];
        fb->buf[i] = fb->buf[i+2];
        fb->buf[i+2] = temp;
    }
    // ESP_LOGD(CAM, "%d, %d, %d",fb->buf[(220*3)], fb->buf[(220*3)+1], fb->buf[(220*3)+2]);
}

void updateJPGBuffer(camera_fb_t *img) {
    ESP_LOGI(CAM, "Updating JPG buffer");

    //fmt2jpg expects BGR format
    if (img->format == PIXFORMAT_RGB888) swapRBchannels(img);

    if (jpgImg.buf) {
        free(jpgImg.buf);
        jpgImg.buf = nullptr;
        jpgImg.len = 0;
    }

    while (!fmt2jpg(img->buf, img->len, img->width, img->height, img->format, 80, &jpgImg.buf, &jpgImg.len)) {
        Serial.println("Format to JPG failed");
        delay(10000);
    }
}

void checkMem(const char* msg, bool level=0) {
    auto& free_size = heap_caps_get_free_size;
    auto& total_size = heap_caps_get_total_size;
    if (level) {
      ESP_LOGW(CAM, "\n[%s]:\n"
                      "\tInternal: %d/%d bytes free - %d%%\n"
                      "\tPSRAM: %d/%d bytes free - %d%%\n"
                      /*"\t32_bit: %d/%d bytes free - %d%%\n"
                      "\t8_bit: %d/%d bytes free - %d%%\n"*/,
        msg,
        free_size(MALLOC_CAP_INTERNAL), total_size(MALLOC_CAP_INTERNAL), free_size(MALLOC_CAP_INTERNAL)*100/total_size(MALLOC_CAP_INTERNAL),
        free_size(MALLOC_CAP_SPIRAM), total_size(MALLOC_CAP_SPIRAM), free_size(MALLOC_CAP_SPIRAM)*100/total_size(MALLOC_CAP_SPIRAM)/*,
        free_size(MALLOC_CAP_32BIT), total_size(MALLOC_CAP_32BIT), free_size(MALLOC_CAP_32BIT)*100/total_size(MALLOC_CAP_32BIT),
        free_size(MALLOC_CAP_8BIT), total_size(MALLOC_CAP_8BIT), free_size(MALLOC_CAP_8BIT)*100/total_size(MALLOC_CAP_8BIT)*/);
    }
    else {
        ESP_LOGV(CAM, "[%s]:\n"
                      "\tInternal: %d/%d bytes free - %d%%\n"
                      "\tPSRAM: %d/%d bytes free - %d%%\n"
                      "\t32_bit: %d/%d bytes free - %d%%\n"
                      "\t8_bit: %d/%d bytes free - %d%%\n",
            msg,
            free_size(MALLOC_CAP_INTERNAL), total_size(MALLOC_CAP_INTERNAL), free_size(MALLOC_CAP_INTERNAL)*100/total_size(MALLOC_CAP_INTERNAL),
            free_size(MALLOC_CAP_SPIRAM), total_size(MALLOC_CAP_SPIRAM), free_size(MALLOC_CAP_SPIRAM)*100/total_size(MALLOC_CAP_SPIRAM),
            free_size(MALLOC_CAP_32BIT), total_size(MALLOC_CAP_32BIT), free_size(MALLOC_CAP_32BIT)*100/total_size(MALLOC_CAP_32BIT),
            free_size(MALLOC_CAP_8BIT), total_size(MALLOC_CAP_8BIT), free_size(MALLOC_CAP_8BIT)*100/total_size(MALLOC_CAP_8BIT));
    }
}

void resetImg(camera_fb_t *img) {
    memset(img->buf, 0, img->len);
}

void testImage(camera_fb_t* img, int pattern) {
    uint8_t* buf = img->buf;
    memset(buf, 0, img->len);
    if (pattern == 0) return;
    if (pattern == 1) {
        if (img->format == PIXFORMAT_RGB888) {
            for (uint32_t y = 0; y < IMG_HEIGHT*IMG_WIDTH; y+= IMG_WIDTH) {
                int d = 5;
                for (int j = 0; j < d; j++) {
                memset(buf+(y+(IMG_WIDTH/d*j))*3, 255, (IMG_WIDTH/2/d)*3);
                }
            }
        }
        else if (img->format == PIXFORMAT_GRAYSCALE) {
            for (uint32_t y = 0; y < IMG_HEIGHT*IMG_WIDTH; y+= IMG_WIDTH) {
                int d = 5;
                for (int j = 0; j < d; j++) {
                memset(buf+(y+(IMG_WIDTH/d*j)), 255, (IMG_WIDTH/2/d));
                }
            }
        }
    }
    else {
        ESP_LOGE(CAM, "Pattern for %d not created yet", pattern);
    }
}

void printImg(camera_fb_t* img) {
    // Chars: . , ' " ^ * : o O Q 0 & % # @    // Extended List: . , ` ' " ^ * : - + = o s x z O Q 0 & % # @ M W $
    const uint8_t charLU[] = " .,'\"^*:oOQ0&%#@"; //size has to be power of two
    uint8_t* buf = img->buf;
    uint8_t b;
    int32_t i, lastChar;
    size_t constexpr bufsize = IMG_WIDTH;
    uint8_t linebuf[IMG_WIDTH];
    for (int32_t x = 0; x < IMG_WIDTH; x++) {
        Serial.print("_");
    }
    for (int32_t y = 0; y < IMG_HEIGHT*IMG_WIDTH; y+= IMG_WIDTH) {
        for (lastChar = IMG_WIDTH-1; lastChar >= 0; lastChar--) {
            if (img->format == PIXFORMAT_RGB888) {
                b = std::max({buf[(y+lastChar)*3], buf[(y+lastChar)*3+1], buf[(y+lastChar)*3+2]});
            }
            else if (img->format == PIXFORMAT_GRAYSCALE) {
                b = buf[y+lastChar];
            }
            if (b >= 16) {
                break;
            }
        }
        
        i = 0;
        for (int32_t x = 0; x <= lastChar; x++) {
            if (img->format == PIXFORMAT_RGB888) {
                b = std::max({buf[(y+x)*3], buf[(y+x)*3+1], buf[(y+x)*3+2]});
            }
            else if (img->format == PIXFORMAT_GRAYSCALE) {
                b = buf[y+x];
            }
            linebuf[i] = charLU[b >> 4];
            if (i >= bufsize) {
                ESP_LOGE(CAM, "lastChar: %d, i: %d", lastChar, i);
                // Serial.write(linebuf, IMG_WIDTH);
                // Serial.println();
                logErrorAndRestart("Linebuf overflow in printImg()");
            }
            i++;
        }
        Serial.write(linebuf, lastChar+1);
        Serial.println();
    }
    for (int32_t x = 0; x < IMG_WIDTH; x++) {
        Serial.print("_");
    }
    Serial.print("\n\n");
}

// IMPLEMENT ADAPTIVE SCALING
template <typename InT, typename OutT> //Don't pass signed types
void scaleCameraBuffer(const camera_fb_t* inImg, camera_fb_t* outImg, bool adaptive=false) {
    if (inImg->len != IMG_WIDTH*IMG_HEIGHT*sizeof(InT) || outImg->len != IMG_WIDTH*IMG_HEIGHT*sizeof(OutT) || inImg->len*sizeof(OutT) != outImg->len*sizeof(InT)) {
        logErrorAndRestart("Recieved incompatible type or buf size");
    }
    resetImg(outImg);
    InT* p1 = reinterpret_cast<InT*>(inImg->buf), *p1_static = p1;
    OutT* p2 = reinterpret_cast<OutT*>(outImg->buf);
    if constexpr (sizeof(InT) <= sizeof(OutT)) {
        for (; p1-p1_static < inImg->len/sizeof(InT); p1++, p2++) {
            *p2 = static_cast<OutT>(*p1) << (8*(sizeof(OutT)-sizeof(InT)));
        }
    }
    else {
        for (; p1-p1_static < inImg->len/sizeof(InT); p1++, p2++) {
            *p2 = static_cast<OutT>(*p1 >> (8*(sizeof(InT)-sizeof(OutT))));
        }
    }


}

template<typename T>
Pixel findMaxPixel(camera_fb_t *img) {
    if constexpr (std::is_same_v<T, uint8_t>) {
        uint8_t *maxptr = std::max_element(img->buf, img->buf+img->len);
        uint32_t length = maxptr - img->buf;
        uint16_t x = length % img->width, y = length / img->width;
        return {x,y,*maxptr};
    }
    else if constexpr (std::is_same_v<T, uint16_t>) {
        uint16_t *start = reinterpret_cast<uint16_t*>(img->buf);
        uint16_t *maxptr = std::max_element(start, start+(img->len/2));
        uint32_t length = maxptr - start;
        uint16_t x = length % img->width, y = length / img->width;
        return {x,y,*maxptr};
    }
    else logErrorAndRestart("Type Not Supported for findMax Pixel");
}

template<typename T>
void pixelsAboveThreshold(camera_fb_t *img, std::vector<Pixel> *pixels, const int t) {
    if (img->format != PIXFORMAT_GRAYSCALE) logErrorAndRestart("Image format to pixelsAboveThreshold not supported");
    T* ptr = reinterpret_cast<T*>(img->buf), *static_ptr = ptr;
    for (; ptr-static_ptr < img->len/sizeof(T); ptr++) {
        if (*ptr > t) {
            int x, y;
            x = (ptr-static_ptr) % IMG_WIDTH;
            y = (ptr-static_ptr) / IMG_WIDTH;
            pixels->emplace_back(x,y,*ptr);
        }
    }
    std::sort(pixels->begin(), pixels->end(), [](const Pixel& a, const Pixel& b) {return a.i > b.i;});
}

void convertToGrayScale(camera_fb_t *pIn, camera_fb_t *pOut, bool weighted=false) {
    if (pIn->len != 3*pIn->width*pIn->height || pOut->len != pOut->width*pOut->height || pOut->width != pIn->width || pOut->height != pIn->height) {
        logErrorAndRestart("Invalid params passed to convertToGrayscale()");
    }
    for (uint8_t *r = pIn->buf, *g=pIn->buf+1, *b=pIn->buf+2, *out = pOut->buf; 
            r-pIn->buf < pIn->len; r+=3, g+=3, b+=3, out++) {

        *out = (*r+*b+*g)/3;
    }
}

void applyMask(camera_fb_t* pInImg, camera_fb_t *pMaskImg, int threshold, const std::array<uint8_t, 3> *color, uint8_t alpha = 100) {
    if (pMaskImg->format != PIXFORMAT_GRAYSCALE) logErrorAndRestart("[applyMask()]: maskImg not grayscale");
    if (pInImg->format != PIXFORMAT_RGB888) logErrorAndRestart("[applyMask()]: Format not supported for pInImg/pOutImg");
    // if (channel > 3) logErrorAndRestart("[applyMask()]: Channel Invalid");

    uint32_t a = alpha;

    uint8_t *pIn = pInImg->buf, *pMask = pMaskImg->buf;
    for (; pMask-pMaskImg->buf < pMaskImg->len; pIn+=3, pMask++) {
        if (*pMask > threshold) {
            if (color == nullptr) {
                pIn[0] = (a * *pMask + (100-a) * pIn[0]) / 100;
                pIn[1] = (a * *pMask + (100-a) * pIn[1]) / 100;
                pIn[2] = (a * *pMask + (100-a) * pIn[2]) / 100;
            }
            else {
                pIn[0] = (a * *pMask * (*color)[0] / 255 + (100-a) * pIn[0]) / 100;
                pIn[1] = (a * *pMask * (*color)[1] / 255 + (100-a) * pIn[1]) / 100;
                pIn[2] = (a * *pMask * (*color)[2] / 255 + (100-a) * pIn[2]) / 100;
            }
        }
    }
} 


void calculateGradBuffer(camera_fb_t *img, camera_fb_t *img2, ImgAreaOfInterest *area = nullptr) {
    resetImg(img2); 
    
    resetImg(&uint16Img);
    uint8_t* rgb = img->buf;
    uint8_t* grad = img2->buf;
    uint16_t* gradE = reinterpret_cast<uint16_t*>(uint16Img.buf);

  
    uint32_t x, x1, x2, y, y1, y2;
    uint32_t x0y0, x0y1, x0y2, x1y0, /*x1y1,*/ x1y2, x2y0, x2y1, x2y2;
    uint16_t maxg = 0;

    size_t minY = 0; // Inclusive
    size_t maxY = IMG_HEIGHT - 3; // Inclusive
    size_t minX = 0; // Inclusive
    size_t maxX = IMG_WIDTH - 3; // Inclusive

    if (area != nullptr) {
        minY = std::max<int>(minY, area->top);
        maxY = std::min<int>(maxY, area->bottom);
        minX = std::max<int>(minX, area->left);
        maxX = std::min<int>(maxX, area->right);
    }
    
    for (y = minY*IMG_WIDTH; y <= maxY*IMG_WIDTH; y+=IMG_WIDTH) {
        y1 = y+IMG_WIDTH;
        y2 = y1+IMG_WIDTH;

        for (x = minX; x <= maxX; x++) {
            x1 = x+1;
            x2 = x1+1;
            x0y0 = (y+x)*3;
            x0y1 = (y1+x)*3;
            x0y2 = (y2+x)*3;
            x1y0 = (y+x1)*3;
            // x1y1 = (y1+x1)*3;
            x1y2 = (y2+x1)*3;
            x2y0 = (y+x2)*3;
            x2y1 = (y1+x2)*3;
            x2y2 = (y2+x2)*3;
      

            auto sobel_gx = [&](uint32_t offset) {
                return abs(-((int16_t)rgb[x0y0+offset]) + ((int16_t)rgb[x2y0+offset])
                        -2*((int16_t)rgb[x0y1+offset]) + 2*((int16_t)rgb[x2y1+offset])
                        -((int16_t)rgb[x0y2+offset]) + ((int16_t)rgb[x2y2+offset]));
            };
            auto sobel_gy = [&](uint32_t offset) {
                return abs(((int16_t)rgb[x0y0+offset]) + 2*((int16_t)rgb[x1y0+offset]) + ((int16_t)rgb[x2y0+offset])
                        -((int16_t)rgb[x0y2+offset]) - 2*((int16_t)rgb[x1y2+offset]) - ((int16_t)rgb[x2y2+offset]));
            };
      
            gradE[y1+x1] = max({
                (sobel_gx(0)+sobel_gy(0))/2,
                (sobel_gx(1)+sobel_gy(1))/2,
                (sobel_gx(2)+sobel_gy(2))/2
            });
      
            maxg = std::max(maxg, gradE[y1+x1]);
        }
    }
    int d = 1;
    while (maxg/d > 256) d++;
  
    for (int i = 0; i < img2->len; i++) grad[i] = static_cast<uint8_t>(gradE[i]/d);
}

void calcCircleCoordinates(int16_t* buf, uint8_t r) {
    // Bresenham's Pixelated Circle Coordinate Finding Algorithm

    // d = 4(x+1)^2+4(y-.5)^2-4r^2
    // d = 4x^2+8x+4 +4y^2-4y+1 -4r^2
    // d2u = 4(x+2)^2 +4(y-.5)^2 -4r^2
    // d2u = 4x^2+16x+16 +4y^2-4y+1 -4r^2
    // d2u-d = 8x+12

    // d2d = 4(x+2)^2 +4(y-1.5)^2 -4r^2
    // d2d = 4x^2+16x+16 +4y^2-12y+9 -4r^2
    // d2d-d = 8x+12 -8y+8
    // d2d-d = 8(x-y)+20

    // d0 = 4(0+1)^2 +4(r-.5)^2 -4r^2
    // d0 = 4 +4r^2-4r+1 -4r^2
    // d0 = -4r+5

    ESP_LOGI(CAM, "Calculating circle coordinates");
    int16_t y=r, x=0, d = 5-4*r;
    while (true) {
        if (d<=0) {
            buf[x]=y;
            d += 8*x+12;
            x++;
        }
        else {
            buf[x] = --y;
            d += 8*(x-y)+20;
            x++;
        }
        if (y<x) {
            break;
        }
    }
    // for (int i = 0; i < r; i++) {
    //   Serial.printf("(%d, %d)\n",i, buf[i]);
    // }
}

Color pixelColor(int r, int g, int b) {
    // if (r > 120 && r-max(g, b) > 50) { 
    //   return Color::Red;
    // }
    // if (max({r,g,b}) < 70 && abs(max({r,g,b})-min({r,g,b})) < 12) { 
    //   return Color::Black;
    // } 
    // if (r == max({r,g,b}) && r - max(g,b) > 10) {
    //   return Color::Brown;
    // }
    
    // return Color::Other;
    HSV hsv(r,g,b);
    // if (IN_RANGE(hsv.h, 240, 360)) return Color::Blue;
    // if (hsv.v < 60 && hsv.s < 60) return Color::Black;
    // if (IN_RANGE(hsv.h, 50, 90) && hsv.s > 100 && IN_RANGE(hsv.v, 90, 140) && r > 120) return Color::Red;

    if (IN_RANGE(hsv.h, 240, 360) && hsv.s > 90) return Color::Blue; //Good on FMS 2 LEDs
    if (IN_RANGE(hsv.h, 45, 80) && hsv.s > 50 && IN_RANGE(hsv.v, 90, 190) && r > 110) return Color::Red;
    if (hsv.v < 60 && hsv.s < 60) return Color::Black;
    



    

    return Color::Other;
}

bool selectColor(camera_fb_t* img, camera_fb_t* colorImg, Color color, ImgAreaOfInterest *areaOfColor = nullptr, bool debug = false) {
    resetImg(colorImg);

    bool foundColor = false;
    if (areaOfColor != nullptr) {
        *areaOfColor = {
            .top = IMG_HEIGHT-1,
            .bottom = 0,
            .left = IMG_WIDTH-1,
            .right = 0
        }; // Starts inverted
    }
    for (size_t i = 0; i < colorImg->len; i+=3) {
        Color c = pixelColor(img->buf[i], img->buf[i+1], img->buf[i+2]);

        if (c == color) {
            foundColor = true;

            if (areaOfColor != nullptr) {
                int x = i/3 % IMG_WIDTH; //TODO: Optimize (incremental tracking)
                int y = i/3 / IMG_WIDTH;

                if (y-5 < areaOfColor->top) areaOfColor->top = y-5;
                if (y+5 > areaOfColor->bottom) areaOfColor->bottom = y+5;
                if (x-5 < areaOfColor->left) areaOfColor->left = x-5;
                if (x+5 > areaOfColor->right) areaOfColor->right = x+5;
            }

            if (!debug) {
                colorImg->buf[i] = 255;
                colorImg->buf[i+1] = 255;
                colorImg->buf[i+2] = 255;
            }
            else {
                // HSV hsv(img->buf[i], img->buf[i+1], img->buf[i+2]);
                // colorImg->buf[i] = hsv.h/2;
                // colorImg->buf[i+1] = hsv.s;

                // colorImg->buf[i+2] = hsv.v;
                colorImg->buf[i] = img->buf[i];
                colorImg->buf[i+1] = img->buf[i+1];
                colorImg->buf[i+2] = img->buf[i+2];
            }
        }

        else {
            if (!debug) {
                colorImg->buf[i] = 0;
                colorImg->buf[i+1] = 0;
                colorImg->buf[i+2] = 0;
            }
            else {
                // HSV hsv(img->buf[i], img->buf[i+1], img->buf[i+2]); 
                // colorImg->buf[i] = hsv.h/2; // division for differentiation with focused parts
                // colorImg->buf[i+1] = hsv.s;
                // colorImg->buf[i+2] = hsv.v;
                colorImg->buf[i] = img->buf[i];
                colorImg->buf[i+1] = img->buf[i+1];
                colorImg->buf[i+2] = img->buf[i+2];
            }
        }
    }
    if (areaOfColor != nullptr) {
        if (areaOfColor->top < 0) areaOfColor->top = 0;
        if (areaOfColor->bottom > IMG_HEIGHT-1) areaOfColor->bottom = IMG_HEIGHT-1;
        if (areaOfColor->left < 0) areaOfColor->left = 0;
        if (areaOfColor->right > IMG_WIDTH-1) areaOfColor->right = IMG_WIDTH-1;
    }
    return foundColor;
}

template <typename T> //uint8_t or uint16_t
void addToPixelIntensity(T *x, uint16_t i) {
    if (std::is_same_v<T, uint8_t>) {
        if (*x+i < 256) *x+=i;
        else *x = 255;
    }
    else if (std::is_same_v<T, uint16_t>) {
        if (*x+i < 65535) *x+=i;
        else *x = 65535;
    }
    else {
        // ESP_LOGE(CAM, "Type passed to \"addToPixelIntensity\" (%s)is not valid", typeid(T).name());
        ESP_LOGE(CAM, "Type passed to \"addToPixelIntensity\" is not valid");
        vTaskDelay(pdMS_TO_TICKS(10000));
        ESP.restart();
    }
}

// OPTIMIZE
template<typename T> //uint8_t or uint16_t
void drawCircle(camera_fb_t *fb, int cx, int cy, int r, uint32_t intensity) {
    if (r < MIN_RAD || r > MAX_RAD || r*2+1 > IMG_WIDTH || r*2+1 > IMG_HEIGHT) {
        ESP_LOGE(CAM, "Radius %d is not supported", r);
        delay(10000);
        ESP.restart();
    }

    if (fb == nullptr || fb->buf == nullptr) {
        ESP_LOGE(CAM, "Passed frame buffer is null");
        vTaskDelay(pdMS_TO_TICKS(10000));
        ESP.restart();
    }

    if (!std::is_same_v<T, uint8_t> && !std::is_same_v<T, uint16_t>) {
        // ESP_LOGE(CAM, "Type passed to \"drawCircle\" (%s)is not valid", typeid(T).name());
        ESP_LOGE(CAM, "Type passed to \"drawCircle\" is not valid");
        vTaskDelay(pdMS_TO_TICKS(10000));
        ESP.restart();
    }
    if (intensity == 0) return;

    T *gray = reinterpret_cast<T*>(fb->buf);
    int16_t *buf = circleCoordinates[r-MIN_RAD];
    if (buf == nullptr) {
        ESP_LOGE(CAM, "circleCoordinates[%d] is null for radius %d (%d, %d)", r-MIN_RAD, r, MIN_RAD, MAX_RAD);
        vTaskDelay(pdMS_TO_TICKS(10000));
        ESP.restart();
    }
  

    int height = fb->height, width = fb->width;
  
    if (cy-r >= 0 && cy+r < height && cx-r >= 0 && cx+r < fb->width) {
        // No chance of out of bounds

        addToPixelIntensity<T>(&gray[(width) * (cy+r) + cx]   ,intensity);
        addToPixelIntensity<T>(&gray[(width) * (cy-r) + cx]   ,intensity);
        addToPixelIntensity<T>(&gray[(width) * (cy)   + cx+r] ,intensity);
        addToPixelIntensity<T>(&gray[(width) * (cy)   + cx-r] ,intensity);

        for (int i = 1; i < r; i++) {
            if (buf[i]<=i) {
                if (buf[i] == i) {
                    addToPixelIntensity<T>(&gray[(width) * (cy+buf[i]) + cx+i]    , intensity);
                    addToPixelIntensity<T>(&gray[(width) * (cy-i)    + cx+buf[i]] , intensity);
                    addToPixelIntensity<T>(&gray[(width) * (cy-buf[i]) + cx-i]    , intensity);
                    addToPixelIntensity<T>(&gray[(width) * (cy+i)    + cx-buf[i]] , intensity);
                }
                break;
            }

            
            addToPixelIntensity<T>(&gray[(width) * (cy+buf[i]) + cx+i]    , intensity);
            addToPixelIntensity<T>(&gray[(width) * (cy+i)    + cx+buf[i]] , intensity);
            addToPixelIntensity<T>(&gray[(width) * (cy-i)    + cx+buf[i]] , intensity);
            addToPixelIntensity<T>(&gray[(width) * (cy-buf[i]) + cx+i]    , intensity);
            addToPixelIntensity<T>(&gray[(width) * (cy-buf[i]) + cx-i]    , intensity);
            addToPixelIntensity<T>(&gray[(width) * (cy-i)    + cx-buf[i]] , intensity);
            addToPixelIntensity<T>(&gray[(width) * (cy+i)    + cx-buf[i]] , intensity);
            addToPixelIntensity<T>(&gray[(width) * (cy+buf[i]) + cx-i]    , intensity);

      
        }
    }
    else {
        // Chance of collision

        // if (cy - r < 0) {
        //   ESP_LOGD(CAM, "cy is too high {%d,%d}", cy, r);
        // }
        // if (cy + r >= fb->height) {
        //   ESP_LOGD(CAM, "cy is too low {%d, %d}", cy, r);
        // }
        // if (cx - r < 0) {
        //   ESP_LOGD(CAM, "cx is too to the left {%d, %d}", cx, r);
        // }
        // if (cx+r >= fb->width) {
        //   ESP_LOGD(CAM, "cx is too to the right {%d, %d}", cx, r);
        // }

        // Top, Botton, Right, and Left points
        if (cy+r < height) addToPixelIntensity<T>(&gray[(width) * (cy+r) + cx] , intensity);
        if (cy-r >= 0)         addToPixelIntensity<T>(&gray[(width) * (cy-r) + cx] , intensity);
        if (cx+r < fb->width)  addToPixelIntensity<T>(&gray[(width) * (cy) + cx+r] , intensity);
        if (cx-r >= 0)         addToPixelIntensity<T>(&gray[(width) * (cy) + cx-r] , intensity);

        // Q1 Top 1/8
        int i = 1;
        if (cx < width-1 && cy > 0) {
            while ((cx + i < 0 || cy - buf[i] < 0) && buf[i] > i) i++;
            for (; cx + i < width && cy - buf[i] < height && buf[i] > i; i++) {
                addToPixelIntensity<T>(&gray[(cy-buf[i]) * width + (cx + i)], intensity);
            }

            // Q1 Side 1/8
            i = 1;
            while ((cx + buf[i] >= width || cy - i >= height) && buf[i] > i) i++;
            for (; cx + buf[i] >= 0 && cy - i >= 0 && buf[i] > i; i++) {
                addToPixelIntensity<T>(&gray[(cy-i) * width + (cx+buf[i])], intensity);
            }

            // Q1 Diagonal
            if (buf[i] == i) {
                addToPixelIntensity<T>(&gray[(cy-i) * width + (cx+buf[i])], intensity);
            }
        }

        // Q2 Top 1/8
        i = 1;
        if (cx > 0 && cy > 0) {
            while ((cx - i >= width || cy - buf[i] < 0) && buf[i] > i) i++;
            for (; cx - i >= 0 && cy - buf[i] < height && buf[i] > i; i++) {
                addToPixelIntensity<T>(&gray[(cy-buf[i]) * width + (cx-i)], intensity);
            }

            // Q2 Side 1/8
            i = 1;
            while ((cx - buf[i] < 0  || cy - i >= height) && buf[i] > i) i++;
            for (; cx - buf[i] < width && cy - i >= 0 && buf[i] > i; i++) {
                addToPixelIntensity<T>(&gray[(cy-i) * width + (cx-buf[i])], intensity);
            }

            // Q2 Diagonal
            if (buf[i] == i) {
                addToPixelIntensity<T>(&gray[(cy-i) * width + (cx-buf[i])], intensity);
            }
        }
        // Q3 Bottom 1/8
        i = 1;
        if (cx > 0 && cy < height-1) {
            while ((cx - i >= width || cy + buf[i] >= height) && buf[i] > i) i++;
            for (; cx - i >= 0 && cy + buf[i] >= 0 && buf[i] > i; i++) {
                addToPixelIntensity<T>(&gray[(cy+buf[i]) * width + (cx-i)], intensity);
            }

            // Q3 Side 1/8
            i = 1;
            while ((cx - buf[i] < 0  || cy + i < 0) && buf[i] > i) i++;
            for (; cx - buf[i] < width && cy + i < height && buf[i] > i; i++) {
                addToPixelIntensity<T>(&gray[(cy+i) * width + (cx-buf[i])], intensity);
            }

            // Q3 Diagonal
            if (buf[i] == i) {
                addToPixelIntensity<T>(&gray[(cy+i) * width + (cx-buf[i])], intensity);
            }
        }
        // Q4 Bottom 1/8
        i = 1;
        if (cx < width-1 && cy < height-1) {
            while ((cx + i < 0 || cy + buf[i] >= height) && buf[i] > i) i++;
            for (; cx + i < width && cy + buf[i] >= 0 && buf[i] > i; i++) {
                addToPixelIntensity<T>(&gray[(cy+buf[i]) * width + (cx+i)], intensity);
            }

            // Q4 Side 1/8
            i = 1;
            while ((cx + buf[i] >= width  || cy + i < 0) && buf[i] > i) i++;
            for (; cx + buf[i] >= 0 && cy + i < height && buf[i] > i; i++) {
                addToPixelIntensity<T>(&gray[(cy+i) * width + (cx+buf[i])], intensity);
            }
            // Q4 Diagonal
            if (buf[i] == i) {
                addToPixelIntensity<T>(&gray[(cy+i) * width + (cx+buf[i])], intensity);
            }
        }
    }
}

void houghTransform(camera_fb_t *pInImg, camera_fb_t *pOutImg, int r, ImgAreaOfInterest *area) {
    resetImg(pOutImg);

    if (pInImg->format != PIXFORMAT_GRAYSCALE && pOutImg->format != PIXFORMAT_GRAYSCALE) {
        ESP_LOGE(CAM, "Hough transform can only be used on grayscale images");
        vTaskDelay(pdMS_TO_TICKS(10000));
        ESP.restart();
    }
    if (pOutImg->len != IMG_WIDTH*IMG_HEIGHT*2) {
        ESP_LOGE(CAM, "Image not long enough");
        vTaskDelay(pdMS_TO_TICKS(10000));
        ESP.restart();
    }
    
    uint8_t *inBuf = pInImg->buf;

    size_t minY = 0;
    size_t maxY = pInImg->height-1;
    size_t minX = 0;
    size_t maxX = pInImg->width-1;

    if (area != nullptr) {
        minY = std::max<int>(minY, area->top);
        maxY = std::min<int>(maxY, area->bottom);
        minX = std::max<int>(minX, area->left);
        maxX = std::min<int>(maxX, area->right);
    }
  
    for (int y = minY*pInImg->width; y <= maxY*pInImg->width; y+=pInImg->width) {
        for (int x = minX; x <= maxX; x++) {
            if (inBuf[y+x] > 0) {
            drawCircle<uint16_t>(pOutImg, x, y/pInImg->width, r, inBuf[y+x]);
            }
        }
    }
}

//minR and maxR are inclusive; pOutmgs should be a pointer to a uint8 camera frame camera_fb_t's
int multipleHoughTransform(camera_fb_t *pInImg, camera_fb_t *pFinalImg, int minR, int maxR, std::vector<std::vector<Pixel>> *pMaxPixels, ImgAreaOfInterest *area) {
    int bestR = minR;
    Pixel maxPixel = {0,0,0}, curPixel;
    for (int r = minR, i = 0; r <= maxR; r++, i++) {
        houghTransform(pInImg, &uint16Img, r, area);
        (*pMaxPixels)[i].reserve(10);
        pixelsAboveThreshold<uint16_t>(&uint16Img, &((*pMaxPixels)[i]), 8'192); // 8192, 12288, 16384, 20480
        
        if (!((*pMaxPixels)[i].empty())) curPixel = (*pMaxPixels)[i][0];
        else curPixel = findMaxPixel<uint16_t>(&uint16Img);
        if (curPixel.i > maxPixel.i) {
            maxPixel = curPixel;
            bestR = r;
            scaleCameraBuffer<uint16_t, uint8_t>(&uint16Img, pFinalImg);
        }
    }

    return bestR;
}

template<typename T>
void drawGuideCircles(camera_fb_t *fb, uint32_t intensity) {
    drawCircle<T>(fb, MAX_RAD, MAX_RAD, MIN_RAD, intensity);
    drawCircle<T>(fb, MAX_RAD, MAX_RAD, MAX_RAD, intensity);
}

void ballsFromPixels(std::vector<std::vector<Pixel>> *pixels, std::vector<Ball> *final_balls) {
    size_t num_pixels = 0;
    for (const auto& inner : *pixels) {
        num_pixels += inner.size();
    }

    static std::vector<Ball> balls; balls.clear(); balls.reserve(num_pixels);
    uint16_t rad = MIN_RAD;    for (const auto &inner : *pixels) {
        for (const auto &pixel : inner) {
            balls.emplace_back(pixel, rad);
        }
        rad++;
        // balls.insert(balls.end(), inner.begin(), inner.end());
    }

    std::sort(balls.begin(), balls.end(), [](const Ball& a, const Ball& b) {return a.i > b.i;});

    final_balls->clear();
    static auto is_overlapping = [](const Ball& a, const Ball& b) {
        if (std::abs(static_cast<int32_t>(a.x)-static_cast<int32_t>(b.x)) < std::max(a.r, b.r) && std::abs(static_cast<int32_t>(a.y) - static_cast<int32_t>(b.y)) < std::max(a.r,b.r)) return true;
        else return false;
    };
    for (const auto& ball : balls) {
        if (final_balls->empty()) {
            final_balls->push_back(ball);
            continue;
        }
        bool is_distinct = true;
        for (const auto &prev_ball : *final_balls) {
            if (is_overlapping(prev_ball, ball)) is_distinct = false;
        }
        if (is_distinct) final_balls->push_back(ball);
    }
}

bool updateClientConnection() {
    if (!client || !client.connected()) {
        client.stop();
        if (clientConnected) {
            Serial.println("Client disconnected");
            clientConnected = false;
        }
        client = server.accept();
    }
    if (client && client.connected()) {
        if (!clientConnected) {
            Serial.println("Client connected");
            clientConnected = true;
        }
        return true;
    }
    else return false;
}

bool sendLogViaWiFi(const char* msg, ...) {
    if (updateClientConnection()) {
        uint8_t msgBuf[100];
        va_list args;
        va_start(args, msg);
        int32_t written = vsnprintf(reinterpret_cast<char*>(msgBuf), sizeof(msgBuf), msg, args) + 1; // +1 for null terminator
        va_end(args);
        client.write("MS", 2); // Message Start
        client.write(0x01); // Log Message Identifier
        client.write(reinterpret_cast<const uint8_t*>(&written), sizeof(written)); //4 bytes (signed because of python)
        client.write(msgBuf, written);
        return true;
    }
    else return false;
}

bool sendImgViaWiFi(camera_fb_t *img) {
    if (updateClientConnection()) {
        updateJPGBuffer(img);
        int32_t len = static_cast<int32_t>(jpgImg.len);
        client.write("MS", 2); // Message Start
        client.write(0x02); // Image Message Identifier
        client.write(reinterpret_cast<const uint8_t*>(&len), sizeof(len)); //4 bytes (signed because of python)
        client.write(jpgImg.buf, len);
        return true;
    }
    else return false;
}

void analyzeImg(camera_fb_t* pFb, camera_fb_t **pOutImg, std::unordered_map<Color, std::vector<Ball>> *pFinal_balls) {
    Timer timer("analyzeImg()");

    static bool initArray = false;
    static std::unordered_map<Color, std::vector<int>> ballCounts;
    static std::unordered_map<Color, int> curBallCount;
    if (!initArray) {
        for (auto [color, _] : *pFinal_balls) {
            ballCounts[color] = {};
        }
        initArray = true;
    }
    
    static std::vector<std::vector<Pixel>> maxPixels(MAX_RAD-MIN_RAD+1); 
    int bestR;
    resetImg(&circleImg);

    timer.recordCheckpoint("setup");
    updateRGBBuffer(pFb);
    timer.recordCheckpoint("updateRGBBuffer");

    memcpy(outputImg.buf, rgb888Img.buf, rgb888Img.len);
    
    Serial.println("Pixels:");
    for (auto& [color, balls] : *pFinal_balls) {
        balls.clear();
        for (auto& pixels : maxPixels) pixels.clear();
        ImgAreaOfInterest areaOfInterest;
        bool foundColor = selectColor(&rgb888Img, &colorImg, color, &areaOfInterest);
        LOG_V("MAIN", "T:%d B: %d L: %d R:%d\n", areaOfInterest.top, areaOfInterest.bottom, areaOfInterest.left, areaOfInterest.right);
        timer.recordCheckpoint("selectColor");

        if (foundColor) {
            calculateGradBuffer(&colorImg, &gradImg, &areaOfInterest);
            timer.recordCheckpoint("calcGradBuffer");

            bestR = multipleHoughTransform(&gradImg, &finalHoughImg, MIN_RAD, MAX_RAD, &maxPixels, &areaOfInterest); //TODO: consider balls from multiple rads
            timer.recordCheckpoint("multipleHoughTransform");

            if (!maxPixels[bestR-MIN_RAD].empty()) {
                ballsFromPixels(&maxPixels, &balls);
                for (auto& ball : balls) drawCircle<uint8_t>(&circleImg, ball.x, ball.y, ball.r, 250);
            }
            else {
                Serial.println("\tNo circles detected");
            }
            applyMask(&outputImg, &gradImg, 120, &(colorShadesMap[color][1]), 70);
            applyMask(&outputImg, &circleImg, 120, &(colorShadesMap[color][0]), 50);
        }
        else {
            LOG_I("MAIN", "No color found. Skipped vision algorighthms");
        }
        

        resetImg(&circleImg);
        timer.recordCheckpoint("Color loop done");
    }
    Serial.println("Balls:");
    for (const auto& [color, balls] : *pFinal_balls) {
        Serial.printf("\t%.*s: %d ball(s)\n", colorStringMap[color].size(), colorStringMap[color].data(), balls.size());
        sendLogViaWiFi("%.*s: %d ball(s)", colorStringMap[color].size(), colorStringMap[color].data(), balls.size());
        for (const auto& ball : balls) {
            Serial.printf("\t\t(%d,%d): %d\n", ball.x, ball.y, ball.i);
        }
    }
    timer.recordCheckpoint("Image Processing Done");

    //Update tracker
    int totalBalls = 0;
    static bool detectedBalls = false;
    for (auto [color, balls] : *pFinal_balls) {
        ballCounts[color].push_back(balls.size());
        totalBalls += balls.size();
    }

    if (totalBalls == 0) {
        if (detectedBalls) {
            for (auto& [color, balls] : ballCounts) {
                balls.pop_back();
                std::sort(balls.begin(), balls.end());
                Serial.printf("Final %.*s: %d", colorStringMap[color].size(), colorStringMap[color].data(), balls[balls.size()/2]);
                sendLogViaWiFi("Final %.*s: %d", colorStringMap[color].size(), colorStringMap[color].data(), balls[balls.size()/2]);
                balls.clear();
            }
            detectedBalls = false;
        }
        for (auto& [color, balls] : ballCounts) {
            balls.clear();
        }

    }
    else {
        detectedBalls = true;
    }
    timer.recordCheckpoint("Tracking");
    drawGuideCircles<uint8_t>(&circleImg, 200);
    applyMask(&outputImg, &circleImg, 120, &(colorShadesMap[Color::Other][1]), 50);
    *pOutImg = &outputImg;
}

void tuneColorThresholds(camera_fb_t *pFb, camera_fb_t **pOutImg, Color color) {
    {MyFuncTimer _t("updateRGBBuffer()");
    updateRGBBuffer(pFb);}

    {MyFuncTimer _t("selectColor()");
    selectColor(&rgb888Img, &colorImg, color);}

    {MyFuncTimer _t("DEBUG selectColor()");
    selectColor(&rgb888Img, &hsvImg, color, nullptr, true);}

    {MyFuncTimer _t("calculateGradBuffer()");
    calculateGradBuffer(&colorImg, &gradImg);}

    {MyFuncTimer _t("applyMask() HSV");
    applyMask(&hsvImg, &gradImg, 120, nullptr);}
    *pOutImg = &hsvImg;
}


void setup() {
    vTaskDelay(pdMS_TO_TICKS(1000));
    Serial.begin(115200);
    // Serial.setDebugOutput(true);
    Logger::init();
    LOG_ADD_TAG("MAIN", LogLevel::VERBOSE);

    ESP_LOGI(CAM, "Began");
    checkMem("Began", 1);

    if (psramFound())   ESP_LOGD(CAM, "PSRAM initialized correctly");
    else {
        ESP_LOGW(CAM, "PSRAM not available"); 
        delay(10000); 
        ESP.restart();
    }

    ESP_LOGD(CAM, "Allocating Img Buffers");
    allocate_camera_fbs({&rgb888Img, &colorImg, &grayImg, &gradImg, &uint16Img, &hsvImg, &circleImg, &finalHoughImg, &outputImg});
    
    ESP_LOGD(CAM, "Allocating circle buffers");
    circleCoordinates = (int16_t**) ps_malloc((MAX_RAD-MIN_RAD+1)*sizeof(int16_t*));
    if (circleCoordinates == NULL) {
        Serial.println("Failed to allocate pointer buffer");
        delay(10000);
        ESP.restart();
    }
    int16_t *data = (int16_t*) ps_malloc((MAX_RAD-MIN_RAD+1)*MAX_RAD*sizeof(int16_t));
    if (data == nullptr) {
        Serial.println("Failed to allocate circle buffer");
        delay(10000);
        ESP.restart();
    }
    memset(data, 0, (MAX_RAD-MIN_RAD+1)*MAX_RAD*sizeof(int16_t));
    for (int i = 0; i <= MAX_RAD-MIN_RAD; i++) {
        circleCoordinates[i] = data+(MAX_RAD*i);
        calcCircleCoordinates(circleCoordinates[i], MIN_RAD+i);
    }
    ESP_LOGE(DEBUG, "circle 3 %p", circleCoordinates+3);

    ESP_LOGD(CAM, "Allocated all buffers");
    checkMem("After Allocation", 1);
    
    init_camera();
    sensor_t *s = esp_camera_sensor_get();
    s->set_vflip(s, 1);
    ESP_LOGD(CAM, "cam_task stack watermark: %d bytes\n", uxTaskGetStackHighWaterMark(NULL));
    ESP_LOGD(CAM, "Camera Setup Sucessfull");
    checkMem("After Camera Setup");

    if (!WiFi.softAP(ssid, password)) {
        ESP_LOGE(CAM, "SoftAP creation failed");
        delay(3000);
        ESP.restart();
    }

    IPAddress IP = WiFi.softAPIP();
    Serial.print("SoftAP IP address: ");
    Serial.println(IP);

    server.begin();
    Serial.println("TCP Server started.");
}

void loop() {
    static camera_fb_t *finalImg;
    static std::unordered_map<Color, std::vector<Ball>> balls = {{Color::Blue, {}}, /*{Color::Black, {}},*/ {Color::Red, {}}};

    MyFuncTimer _t("LOOP");
    // ESP_LOGI(CAM, "LOOPING");
    ESP_LOGV(CAM, "cam_task stack watermark: %d bytes\n", uxTaskGetStackHighWaterMark(NULL));

    camera_fb_t *fb;
    {MyFuncTimer _t("camera_capture()");
    fb = camera_capture();}
    analyzeImg(fb, &finalImg, &balls);
    // tuneColorThresholds(fb, &finalImg, Color::Blue);

    ESP_LOGD(CAM, "Finished processing");
    checkMem("After Processing");

    if (sendImgViaWiFi(finalImg)) {
        ESP_LOGD(CAM, "Sent image via WiFi");
    }
    else {
        ESP_LOGD(CAM, "No client, delaying ...");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    esp_camera_fb_return(fb);
    vTaskDelay(pdMS_TO_TICKS(50)); 
    checkMem("After Delay");
}