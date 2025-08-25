#ifndef UTIL
#define UTIL

// typedef enum {
//     COLOR_UNKNOWN = 0,
//     COLOR_BLUE = 1,
//     COLOR_ORANGE = 2
// } Cvec3;

typedef int32_t Cvec3;
const Cvec3 COLOR_UNKNOWN = 0;
const Cvec3 COLOR_BLUE = 1;
const Cvec3 COLOR_ORANGE = 2;


// White balance correction
void white_correct(const float raw[3], const float white[3], float corrected[3]) {
  corrected[0] = raw[0] / white[0];
  corrected[1] = raw[1] / white[1];
  corrected[2] = raw[2] / white[2];

  // Normalize to [0,1]
  float max_component = corrected[0];
  if (corrected[1] > max_component)
    max_component = corrected[1];
  if (corrected[2] > max_component)
    max_component = corrected[2];

  if (max_component > 0.0f) {
    corrected[0] /= max_component;
    corrected[1] /= max_component;
    corrected[2] /= max_component;
  }

  // printf("%.3f, %.3f, %.3f\n", corrected[0], corrected[1], corrected[2]);
}

Cvec3 detect_color(const float rgb_raw[3], const float white[3]) {
    float rgb[3];
    white_correct(rgb_raw, white, rgb);

    float r = rgb[0];
    float g = rgb[1];
    float b = rgb[2];

    // Find min and max
    float max = r;
    if (g > max) max = g;
    if (b > max) max = b;

    float min = r;
    if (g < min) min = g;
    if (b < min) min = b;

    float delta = max - min;

    float h = 0.0f;
    float s = 0.0f;
    float v = max;

    if (delta > 0.00001f) {
        s = (max > 0.0f) ? (delta / max) : 0.0f;

        if (max == r) {
            h = 60.0f * ((g - b) / delta);
            if (h < 0.0f) h += 360.0f;
        } else if (max == g) {
            h = 60.0f * (((b - r) / delta) + 2.0f);
        } else {
            h = 60.0f * (((r - g) / delta) + 4.0f);
        }
    }

    // printf("%.3f, %.3f, %.3f\n", h, s, v);

    // Require saturation & brightness
    if (s > 0.45f && v > 0.3f) {
      // printf("Uh yes papa\n");
        // Blue range
        if (h >= 200.0f && h <= 250.0f) {
          // printf("le blue\n");
          return COLOR_BLUE;
        }
        // Orange (shifted into red zone for this sensor)
        if ((h >= 0.0f && h <= 50.0f) || (h >= 340.0f && h <= 360.0f)) {
            // printf("l'orange\n");
            return COLOR_ORANGE;
        }
    }

    // printf("No papa\n");
    return COLOR_UNKNOWN;
}


#endif /* UTIL */
