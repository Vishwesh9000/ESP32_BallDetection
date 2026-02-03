#define CAMERA_MODEL_WROVER_KIT
#include "esp_camera.h"
#include "cameraPins.h"

#include <WiFi.h>

#include <algorithm>
#include <cmath>
#include <initializer_list>
#include <type_traits>
#include <vector>
#include <utility>
#include <tuple>


extern "C" {
    #include "esp_log.h"
}

#define IMG_WIDTH 320
#define IMG_HEIGHT 240

#define MIN_RAD 25 //Inclusive
#define MAX_RAD 40 //Inclusive

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

    .jpeg_quality = 12, //0-63, for OV series camera sensors, lower number means higher quality
    .fb_count = 2, //When jpeg mode is used, if fb_count more than one, the driver will work in continuous mode.
    .fb_location = CAMERA_FB_IN_PSRAM,
    .grab_mode = CAMERA_GRAB_LATEST//. Sets when buffers should be filled CAMERA_GRAB_WHEN_EMPTY CAMERA_GRAB_LATEST
    // .sccb_i2c_port = -1
};


enum class Color {Red, Blue, Black, Brown, Other};
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
    uint16_t x,y,i;
};

struct Ball {
    uint16_t x,y,r;
    Color color;
};

camera_fb_t rgb888Img = {.buf=nullptr, .len=IMG_HEIGHT*IMG_WIDTH*3, .width=IMG_WIDTH, .height=IMG_HEIGHT, .format=PIXFORMAT_RGB888, .timestamp={0,0}};
camera_fb_t blueImg = {.buf=nullptr, .len=IMG_HEIGHT*IMG_WIDTH*3, .width=IMG_WIDTH, .height=IMG_HEIGHT, .format=PIXFORMAT_RGB888, .timestamp={0,0}};
camera_fb_t grayImg = {.buf=nullptr, .len=IMG_HEIGHT*IMG_WIDTH, .width=IMG_WIDTH, .height=IMG_HEIGHT, .format=PIXFORMAT_GRAYSCALE, .timestamp={0,0}};
camera_fb_t gradImg = {.buf=nullptr, .len=IMG_HEIGHT*IMG_WIDTH, .width=IMG_WIDTH, .height=IMG_HEIGHT,.format=PIXFORMAT_GRAYSCALE, .timestamp={0,0}};
camera_fb_t extendedImg = {.buf=nullptr, .len=IMG_HEIGHT*IMG_WIDTH*2, .width=IMG_WIDTH, .height=IMG_HEIGHT,.format=PIXFORMAT_GRAYSCALE, .timestamp={0,0}};
camera_fb_t jpgImg = {.buf=nullptr, .len=0, .width=IMG_WIDTH, .height=IMG_HEIGHT, .format=PIXFORMAT_JPEG, .timestamp={0,0}};

camera_fb_t hsvImg = {.buf=nullptr, .len=IMG_HEIGHT*IMG_WIDTH*3, .width=IMG_WIDTH, .height=IMG_HEIGHT, .format=PIXFORMAT_RGB888, .timestamp={0,0}};
camera_fb_t circleImg = {.buf=nullptr, .len=IMG_HEIGHT*IMG_WIDTH, .width=IMG_WIDTH, .height=IMG_HEIGHT, .format=PIXFORMAT_GRAYSCALE, .timestamp={0,0}};

camera_fb_t houghImgs[MAX_RAD-MIN_RAD+1];
static constexpr camera_fb_t houghImgTemplate = {.buf=nullptr, .len=IMG_HEIGHT*IMG_WIDTH*2, .width=IMG_WIDTH, .height=IMG_HEIGHT,.format=PIXFORMAT_GRAYSCALE, .timestamp={0,0}};
camera_fb_t finalHoughImg = {.buf=nullptr, .len=houghImgTemplate.len/2, .width=houghImgTemplate.width, .height=houghImgTemplate.height,.format=houghImgTemplate.format, .timestamp={0,0}};
camera_fb_t houghImgTotal = {.buf=nullptr, .len=(MAX_RAD-MIN_RAD+1)*houghImgTemplate.len, .width=houghImgTemplate.width, .height=houghImgTemplate.height,.format=houghImgTemplate.format, .timestamp={0,0}};


int16_t **circleCoordinates;

Mode mode = Mode::ShowColors;

WiFiServer server(8888);
WiFiClient client;


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

void calculateGradBuffer(camera_fb_t *img, camera_fb_t *img2) {
    uint8_t* rgb = img->buf;
    uint8_t* grad = img2->buf;
    uint16_t* gradE = reinterpret_cast<uint16_t*>(extendedImg.buf);

    memset(grad, 0, img2->len);
  
    uint32_t x, x1, x2, y, y1, y2;
    uint32_t x0y0, x0y1, x0y2, x1y0, /*x1y1,*/ x1y2, x2y0, x2y1, x2y2;
    uint16_t maxg = 0;

    for (y = 0; y < IMG_WIDTH*(IMG_HEIGHT-2); y+=IMG_WIDTH) {
        y1 = y+IMG_WIDTH;
        y2 = y1+IMG_WIDTH;

        for (x = 0; x < IMG_WIDTH-2; x++) {
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
  
    for (int i = 0; i < img2->len; i++) grad[i] = (uint8_t) (gradE[i]/d);
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
        // ESP_LOGV(CAM, "Drawing non-colliding circle");

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

            ESP_LOGV(CAM, "Drawing Coordinate (%d, %d)", i, buf[i]);
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
        // ESP_LOGV(CAM, "Drawing colliding circle");

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

void houghTransform(camera_fb_t *pInImg, camera_fb_t *pOutImg, int r) {
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
  
    memset(pOutImg->buf, 0, pOutImg->len);
  
    uint8_t *inBuf = pInImg->buf;
  
    for (int y = 0; y < pInImg->height; y++) {
        for (int x = 0; x < pInImg->width; x++) {
            if (inBuf[(pInImg->width)*y+x] > 0) {
            drawCircle<uint16_t>(pOutImg, x, y, r, inBuf[(pInImg->width)*y+x]);
            }
        }
    }

    // Normalizing to uint8_t
    // uint16_t *in = reinterpret_cast<uint16_t*>(extendedImg.buf), *inStart = in; uint8_t *out = pOutImg->buf;
    // int maxVal = *std::max_element(in, in+(extendedImg.len/2));
    // int d = std::max(maxVal/255 + ((maxVal/255)*maxVal < 256 ? 0 : 1), 1);
    
    // for (; in - inStart < extendedImg.len/2 && out - pOutImg->buf < pOutImg->len; in++, out++) {
    //   if (*in / d >= 256) {
    //     ESP_LOGE(CAM, "Value exceeds uint8_t max");
    //     vTaskDelay(pdMS_TO_TICKS(10000));
    //     ESP.restart(); 
    //   }
    //   *out = static_cast<uint8_t>(*in/d);
    // }
}

//TODO: Implement ptrs instead of returns
//minR and maxR are inclusive; pOutmgs should be a pointer to maxR-minR+1 camera_fb_t's
int multipleHoughTransform(camera_fb_t *pInImg, camera_fb_t *pOutImgs, int minR, int maxR, std::vector<std::vector<Pixel>> *pMaxPixels) {
    for (int r = minR, i = 0; r <= maxR; r++, i++) {
        houghTransform(pInImg, pOutImgs+i, r);
        (*pMaxPixels)[i].reserve(10);
        pixelsAboveThreshold<uint16_t>(pOutImgs+i, &((*pMaxPixels)[i]), 16384); // 8192, 12288, 16384, 20480
    }
    auto maxPixel = std::max_element(pMaxPixels->begin(), pMaxPixels->end(), [](const std::vector<Pixel>& a, const std::vector<Pixel>& b) {
        if (b.empty()) return false; // a can never be less than empty b
        if (a.empty()) return true;
        return a[0].i<b[0].i;});
    int bestR = maxPixel-pMaxPixels->begin()+minR;
    return bestR;
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
    if (IN_RANGE(hsv.h, 240, 360)) return Color::Blue;
    // if (hsv.v<60 && hsv.s < 80) return Color::Black;



    // if (IN_RANGE(hsv.h, 50, 130) && hsv.s > 100 && IN_RANGE(hsv.v, 50, 140)) return Color::Red;
    // if (IN_RANGE(hsv.h, 50, 90) && hsv.s > 100 && IN_RANGE(hsv.v, 90, 140)) return Color::Red; //Good Lamp Light Home Phone Flashlight


    // if (IN_RANGE(hsv.h, 50, 90) && hsv.s > 90 && IN_RANGE(hsv.v, 90, 140)) return Color::Red;
    //OG
    // if (IN_RANGE(hsv.h, 50, 90) && hsv.s > 90 && IN_RANGE(hsv.v, 60, 140)) return Color::Red;

    // V1
    if (IN_RANGE(hsv.h, 50, 90) && hsv.s > 90 && IN_RANGE(hsv.v, 60, 140)) return Color::Red;

    // V2
    // Extremely high saturation
    // if (hsv.s x> 160) return Color::Red; 
    //Way to selective (balls not round) + tons of bg noise

    // if (IN_RANGE(hsv.h, 50, 90) && hsv.s > 90 && IN_RANGE(hsv.v, 60, 140)) return Color::Red;

    
    // if (hsv.h > 60 && hsv.h < 120 && hsv.s > 40 && hsv.v > 40 && hsv.v < 100) return Color::Brown;

    return Color::Other;
}

void selectColors(camera_fb_t* img, camera_fb_t* colorImg, Color color, bool debug = false) {
    memset(colorImg->buf, 0, colorImg->len);

    for (int i = 0; i < colorImg->len; i+=3) {
        Color c = pixelColor(img->buf[i], img->buf[i+1], img->buf[i+2]);

        if (c == color) {
            if (!debug) {
                colorImg->buf[i] = 255;
                colorImg->buf[i+1] = 255;
                colorImg->buf[i+2] = 255;
            }
            else {
                HSV hsv(img->buf[i], img->buf[i+1], img->buf[i+2]);
                colorImg->buf[i] = hsv.h/2;
                colorImg->buf[i+1] = hsv.s;
                colorImg->buf[i+2] = hsv.v;
            }
        }

        else {
            if (!debug) {
                colorImg->buf[i] = 0;
                colorImg->buf[i+1] = 0;
                colorImg->buf[i+2] = 0;
            }
        else {
            HSV hsv(img->buf[i], img->buf[i+1], img->buf[i+2]); 
            colorImg->buf[i] = hsv.h/4; // division for differentiation with focused parts
            colorImg->buf[i+1] = hsv.s/2;
            colorImg->buf[i+2] = hsv.v/2;
        }

        }
    }
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

void printImg(camera_fb_t* img, bool optimized=false) {
    // Chars: . , ` ' " ^ * : o O Q 0 & % # @    // Extended List: . , ` ' " ^ * : - + = o s x z O Q 0 & % # @ M W $
    uint8_t* buf = img->buf;

    if (!optimized) {
        for (uint32_t y = 0; y < IMG_HEIGHT*IMG_WIDTH; y+= IMG_WIDTH) {
            for (uint32_t x = 0; x < IMG_WIDTH; x++) {
                if (img->format == PIXFORMAT_RGB888) {
                    uint8_t i = std::max({buf[(y+x)*3], buf[(y+x)*3+1], buf[(y+x)*3+2]});
                    if (i == 0) Serial.print(" ");
                    else if (i <= 16) Serial.print(".");
                    else if (i <= 32) Serial.print(",");
                    else if (i <= 48) Serial.print("`");
                    else if (i <= 64) Serial.print("'");
                    else if (i <= 80) Serial.print("\"");
                    else if (i <= 96) Serial.print("^");
                    else if (i <= 112) Serial.print("*");
                    else if (i <= 128) Serial.print(":");
                    else if (i <= 144) Serial.print("o");
                    else if (i <= 160) Serial.print("O");
                    else if (i <= 186) Serial.print("Q");
                    else if (i <= 192) Serial.print("0");
                    else if (i <= 208) Serial.print("&");
                    else if (i <= 224) Serial.print("%");
                    else if (i <= 240) Serial.print("#");
                    else Serial.print("@");
                }
                else if (img->format == PIXFORMAT_GRAYSCALE) {
                    uint8_t i = buf[y+x];
                    if (i == 0) Serial.print(" ");
                    else if (i <= 16) Serial.print(".");
                    else if (i <= 32) Serial.print(",");
                    else if (i <= 48) Serial.print("`");
                    else if (i <= 64) Serial.print("'");
                    else if (i <= 80) Serial.print("\"");
                    else if (i <= 96) Serial.print("^");
                    else if (i <= 112) Serial.print("*");
                    else if (i <= 128) Serial.print(":");
                    else if (i <= 144) Serial.print("o");
                    else if (i <= 160) Serial.print("O");
                    else if (i <= 186) Serial.print("Q");
                    else if (i <= 192) Serial.print("0");
                    else if (i <= 208) Serial.print("&");
                    else if (i <= 224) Serial.print("%");
                    else if (i <= 240) Serial.print("#");
                    else Serial.print("@");
                }
            }
            Serial.println();
        }
    }
    else {
        for (uint32_t y = 0; y < IMG_HEIGHT*IMG_WIDTH; y+= IMG_WIDTH) {
            for (uint32_t x = 0; x < IMG_WIDTH; x++) {
                if (img->format == PIXFORMAT_RGB888) {
                    uint8_t i = std::max({buf[(y+x)*3], buf[(y+x)*3+1], buf[(y+x)*3+2]});
                    if (i == 0) Serial.print(" ");
                    else if (i <= 16) Serial.print(".");
                    else if (i <= 32) Serial.print(",");
                    else if (i <= 48) Serial.print("`");
                    else if (i <= 64) Serial.print("'");
                    else if (i <= 80) Serial.print("\"");
                    else if (i <= 96) Serial.print("^");
                    else if (i <= 112) Serial.print("*");
                    else if (i <= 128) Serial.print(":");
                    else if (i <= 144) Serial.print("o");
                    else if (i <= 160) Serial.print("O");
                    else if (i <= 186) Serial.print("Q");
                    else if (i <= 192) Serial.print("0");
                    else if (i <= 208) Serial.print("&");
                    else if (i <= 224) Serial.print("%");
                    else if (i <= 240) Serial.print("#");
                    else Serial.print("@");
                }
                else if (img->format == PIXFORMAT_GRAYSCALE) {
                    uint8_t i = buf[y+x];
                    if (i == 0) Serial.print(" ");
                    else if (i <= 16) Serial.print(".");
                    else if (i <= 32) Serial.print(",");
                    else if (i <= 48) Serial.print("`");
                    else if (i <= 64) Serial.print("'");
                    else if (i <= 80) Serial.print("\"");
                    else if (i <= 96) Serial.print("^");
                    else if (i <= 112) Serial.print("*");
                    else if (i <= 128) Serial.print(":");
                    else if (i <= 144) Serial.print("o");
                    else if (i <= 160) Serial.print("O");
                    else if (i <= 186) Serial.print("Q");
                    else if (i <= 192) Serial.print("0");
                    else if (i <= 208) Serial.print("&");
                    else if (i <= 224) Serial.print("%");
                    else if (i <= 240) Serial.print("#");
                    else Serial.print("@");
                }
            }
            Serial.println();
        }
    }
    Serial.print("\n\n");
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

// IMPLEMENT ADAPTIVE SCALING
template <typename InT, typename OutT> //Don't pass signed types
void scaleCameraBuffer(const camera_fb_t* inImg, camera_fb_t* outImg, bool adaptive=false) {
    if (inImg->len != IMG_WIDTH*IMG_HEIGHT*sizeof(InT) || outImg->len != IMG_WIDTH*IMG_HEIGHT*sizeof(OutT) || inImg->len*sizeof(OutT) != outImg->len*sizeof(InT)) {
        logErrorAndRestart("Recieved incompatible type or buf size");
    }
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

void logErrorAndRestart(const char* s) {
    ESP_LOGE(ERROR, "%s", s); 
    vTaskDelay(pdMS_TO_TICKS(10000));
    ESP.restart();
}

template<typename T>
void drawGuideCircles(camera_fb_t *fb, uint32_t intensity) {
    drawCircle<T>(fb, MAX_RAD, MAX_RAD, MIN_RAD, intensity);
    drawCircle<T>(fb, MAX_RAD, MAX_RAD, MAX_RAD, intensity);
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

void applyMask(camera_fb_t* pInImg, camera_fb_t *pMaskImg, int threshold, uint8_t channel = 3) {
    if (pMaskImg->format != PIXFORMAT_GRAYSCALE) logErrorAndRestart("[applyMask()]: maskImg not grayscale");
    if (pInImg->format != PIXFORMAT_RGB888) logErrorAndRestart("[applyMask()]: Format not supported for pInImg/pOutImg");
    if (channel > 3) logErrorAndRestart("[applyMask()]: Channel Invalid");
    uint8_t *pIn = pInImg->buf, *pMask = pMaskImg->buf;
    for (; pMask-pMaskImg->buf < pMaskImg->len; pIn+=3, pMask++) {
        if (*pMask > threshold) {
            if (channel == 3) {
                pIn[0] = *pMask;
                pIn[1] = *pMask;
                pIn[2] = *pMask;
            }
            else {
                pIn[channel] = *pMask;
            }
        }
    }
} 

void findBalls(std::vector<std::vector<Pixel>> *pixels, std::vector<Pixel> *final_balls) {
    size_t num_pixels = 0;
    for (const auto& inner : *pixels) {
        num_pixels += inner.size();
    }

    static std::vector<Pixel> balls; 
    balls.clear(); balls.reserve(num_pixels);
    for (const auto &inner : *pixels) {
        balls.insert(balls.end(), inner.begin(), inner.end());
    }

    std::sort(balls.begin(), balls.end(), [](const Pixel& a, const Pixel& b) {return a.i > b.i;});

    final_balls->clear();
    static auto is_overlapping = [](const Pixel& a, const Pixel& b) {
        if (std::abs(static_cast<int32_t>(a.x)-static_cast<int32_t>(b.x)) < MIN_RAD && std::abs(static_cast<int32_t>(a.y) - static_cast<int32_t>(b.y)) < MIN_RAD) return true;
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

void resetImg(camera_fb_t *img) {
    memset(img->buf, 0, img->len);
}

void analyzeImg(camera_fb_t* pFb, camera_fb_t **pOutImg, std::vector<Pixel> *balls) {
    balls->clear();
    
    {MyFuncTimer _t("updateRGBBuffer()");
    updateRGBBuffer(pFb);}

    {MyFuncTimer _t("selectColors()");
    selectColors(&rgb888Img, &blueImg, Color::Blue);}

    {MyFuncTimer _t("calculateGradBuffer()");
    calculateGradBuffer(&blueImg, &gradImg);}

    static std::vector<std::vector<Pixel>> maxPixels; int bestR;
    maxPixels.clear(); 
    maxPixels.resize(MAX_RAD-MIN_RAD+1);
    {MyFuncTimer _t("multipleHoughTransform()");
    bestR = multipleHoughTransform(&gradImg, &(houghImgs[0]), MIN_RAD, MAX_RAD, &maxPixels);}
    if (!maxPixels[bestR-MIN_RAD].empty()) {
        Pixel maxPixel = maxPixels[bestR-MIN_RAD][0];
        ESP_LOGI(CAM, "BestR: %d, Max Pixel: (%d, %d, %d)", bestR, maxPixel.x, maxPixel.y, maxPixel.i);
        for (int r = MIN_RAD; r <= MAX_RAD; r++) {
            if (maxPixels[r-MIN_RAD].empty()) continue;
            Serial.printf("Radius %d:\n", r);
            for (const Pixel& p : maxPixels[r-MIN_RAD]) {
                Serial.printf("\t(%d, %d): %d\n", p.x, p.y, p.i);
            }
            Serial.print("\n");
        }
        Serial.print("\n");

        findBalls(&maxPixels, balls);
        Serial.println("Balls found:");
        for (const auto& ball : *balls) {
            Serial.printf("\t(%d, %d): %d\n", ball.x, ball.y, ball.i);
        }
        {MyFuncTimer _t("scaleCameraBuffer");
        scaleCameraBuffer<uint16_t, uint8_t>(houghImgs+(bestR-MIN_RAD), &finalHoughImg);}

        // convertToGrayScale(&rgb888Img, &grayImg);
        applyMask(&rgb888Img, &gradImg, 120, 0);
        
        for (const auto& ball : *balls) {
            drawCircle<uint8_t>(&circleImg, ball.x, ball.y, bestR, 200);
        }
        drawGuideCircles<uint8_t>(&circleImg, 200);
        applyMask(&rgb888Img, &circleImg, 120, 1);
        *pOutImg = &rgb888Img;
    }
    else {
        Serial.println("No circles detected");
        applyMask(&rgb888Img, &gradImg, 120, 0);
        drawGuideCircles<uint8_t>(&circleImg, 200);
        applyMask(&rgb888Img, &circleImg, 120, 1);
        *pOutImg = &rgb888Img;
    }
}

void tuneColorThresholds(camera_fb_t *pFb, camera_fb_t **pOutImg) {
    {MyFuncTimer _t("updateRGBBuffer()");
    updateRGBBuffer(pFb);}

    {MyFuncTimer _t("selectColors()");
    selectColors(&rgb888Img, &blueImg, Color::Blue);}

    {MyFuncTimer _t("DEBUG selectColors()");
    selectColors(&rgb888Img, &hsvImg, Color::Blue, true);}

    {MyFuncTimer _t("calculateGradBuffer()");
    calculateGradBuffer(&blueImg, &gradImg);}

    {MyFuncTimer _t("applyMask() HSV");
    applyMask(&hsvImg, &gradImg, 120, 3);}
    *pOutImg = &hsvImg;
}

void setup() {
    vTaskDelay(pdMS_TO_TICKS(1000));
    Serial.begin(115200);

    // Serial.setDebugOutput(true);
    esp_log_level_set("*", ESP_LOG_VERBOSE);

    ESP_LOGI(CAM, "Began");
    checkMem("Began", 1);

    if (psramFound())   ESP_LOGD(CAM, "PSRAM initialized correctly");
    else                {ESP_LOGW(CAM, "PSRAM not available"); delay(10000); ESP.restart();}

    ESP_LOGD(CAM, "Allocating Img Buffers");
    allocate_camera_fbs({&rgb888Img, &blueImg, &gradImg, &extendedImg, &houghImgTotal, &finalHoughImg, &grayImg, &hsvImg, &circleImg});
    
    ESP_LOGD(CAM, "Allocating HoughImgs");
    for (int i = 0; camera_fb_t& fb : houghImgs) {
        fb = houghImgTemplate;
        fb.buf = houghImgTotal.buf + (i*houghImgTemplate.len);
        i++;
    }
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
    static std::vector<Pixel> balls; balls.clear();
    resetImg(&circleImg);
    MyFuncTimer _t("LOOP");
    // ESP_LOGI(CAM, "LOOPING");
    ESP_LOGV(CAM, "cam_task stack watermark: %d bytes\n", uxTaskGetStackHighWaterMark(NULL));

    camera_fb_t *fb;
    {MyFuncTimer _t("camera_capture()");
    fb = camera_capture();}

    analyzeImg(fb, &finalImg, &balls);
    // tuneColorThresholds(fb, &finalImg);




    ESP_LOGD(CAM, "Finished processing");
    checkMem("After Processing");
    if (!(client && client.connected())) {
        client = server.accept();
        if (client) {
            Serial.println("Client Connected");
        }
        else {
            {MyFuncTimer _t("Normal printImg");
            printImg(&finalHoughImg);}
            ESP_LOGI(CAM, "No client, delaying ...");
            vTaskDelay(pdMS_TO_TICKS(2000));
        }
    }
    if (client && client.connected()) {
        // client.write(fb->buf, fb->len);
        // delay(1000);
        
        updateJPGBuffer(finalImg);
        ESP_LOGD(CAM, "Updated JPG");
        client.write(jpgImg.buf, jpgImg.len);
        ESP_LOGD(CAM, "Sent image via WiFi");
        // delay(1000);
        // updateJPGBuffer(&houghImg);
        // client.write(jpgImg.buf, jpgImg.len);
    }

    esp_camera_fb_return(fb);
    vTaskDelay(pdMS_TO_TICKS(3000)); 
    checkMem("After Delay");
}