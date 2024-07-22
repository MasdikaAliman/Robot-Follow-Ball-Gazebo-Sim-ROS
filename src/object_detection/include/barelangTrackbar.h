#ifndef BARELANG_TRACKBAR_H
#define BARELANG_TRACKBAR_H

#include <opencv2/highgui.hpp>
#include <iostream>

namespace barelang
{
    enum ColorChannel
    {
        RGB,
        HSV,
        HLS,
        Luv,
        Lab
    };
    class Trackbar
    {
    private:
        int *value;
        std::string trackbarName, windowName;
        bool showTrackbar = true;

        static void trackbarCallback(int value, void *userdata) {}

    public:
        Trackbar() = default;
        void initTrackbar(std::string trackbar, std::string window, int &n, int range, bool showTrackbar);
        void initTrackbar(std::string trackbar, std::string window, int *n, int range, bool showTrackbar);
        void setValue(int newValue);
        int getValue();
        operator int();
    };

    class TrackbarMinMax
    {
    private:
        int *minValue, *maxValue;

        bool showTrackbar = false;
        std::string trackbarMinName, trackbarMaxName, windowName;

        static void trackbarMaxCallbackStatic(int value, void *userdata);
        static void trackbarMinCallbackStatic(int value, void *userdata);
        void trackbarMaxCallback();
        void trackbarMinCallback();

    public:
        TrackbarMinMax() = default;
        ~TrackbarMinMax() = default;

        TrackbarMinMax(std::string trackbar, std::string window, int &lo, int &hi, int range, bool showTrackbar);

        TrackbarMinMax(std::string trackbar, std::string window, int *lo, int *hi, int range, bool showTrackbar);

        void initTrackbar(std::string trackbar, std::string window, int &lo, int &hi, int range, bool showTrackbar);

        void setValue(int minVal, int maxVal);
        void setMax(int value);
        void setMin(int value);

        int getMax();
        int getMin();
    };

    class TrackbarColour
    {
    private:
        TrackbarMinMax *Channel1, *Channel2, *Channel3;

    public:
        TrackbarColour();
        TrackbarColour(std::string trackbar, std::string window, ColorChannel color_channel, int &Ch1_hi, int &Ch1_lo, int &Ch2_hi, int &Ch2_lo, int &Ch3_hi, int &Ch3_lo, int range, bool showTrackbar);
        TrackbarColour(std::string trackbar, std::string window, ColorChannel color_channel, int *Ch1_hi, int *Ch1_lo, int *Ch2_hi, int *Ch2_lo, int *Ch3_hi, int *Ch3_lo, int range, bool showTrackbar);
        void initTrackbarColour(std::string trackbar, std::string window, ColorChannel color_channel, int &Ch1_hi, int &Ch1_lo, int &Ch2_hi, int &Ch2_lo, int &Ch3_hi, int &Ch3_lo, int range, bool showTrackbar);

        void setLower(int ch1, int ch2, int ch3);
        void setUpper(int ch1, int ch2, int ch3);

        cv::Scalar getLower();
        cv::Scalar getUpper();
    };
}

#endif //
