#include "barelangTrackbar.h"
#include "icecream.hpp"

using namespace barelang;

void Trackbar::initTrackbar(std::string trackbar, std::string window, int &n, int range, bool showTrackbar)
{
    this->trackbarName = trackbar;
    this->windowName = window;
    this->value = &n;
    this->showTrackbar = showTrackbar;

    if (showTrackbar)
    {
        cv::namedWindow(this->windowName, cv::WINDOW_NORMAL);
        cv::createTrackbar(this->trackbarName, this->windowName, this->value, range);
    }
}

void Trackbar::initTrackbar(std::string trackbar, std::string window, int *n, int range, bool showTrackbar)
{
    this->trackbarName = trackbar;
    this->windowName = window;
    this->value = n;
    this->showTrackbar = showTrackbar;

    if (showTrackbar)
    {
        cv::namedWindow(this->windowName, cv::WINDOW_NORMAL);
        cv::createTrackbar(this->trackbarName, this->windowName, this->value, range);
    }
}

void Trackbar::setValue(int newValue)
{
    if (showTrackbar)
        cv::setTrackbarPos(this->trackbarName, this->windowName, newValue);
    *this->value = newValue;
}

int Trackbar::getValue()
{
    return *this->value;
}

Trackbar::operator int()
{
    return *this->value;
}

TrackbarMinMax::TrackbarMinMax(std::string trackbar, std::string window, int &lo, int &hi, int range, bool showTrackbar)
{
    this->windowName = window;

    this->trackbarMinName = trackbar + " Min";

    this->trackbarMaxName = trackbar + " Max";

    this->minValue = &lo;
    this->maxValue = &hi;

    if (showTrackbar)
    {
        cv::namedWindow(this->windowName, cv::WINDOW_NORMAL);

        cv::createTrackbar(this->trackbarMinName, this->windowName, this->minValue, range, TrackbarMinMax::trackbarMinCallbackStatic, this);

        cv::createTrackbar(this->trackbarMaxName, this->windowName, this->maxValue, range, TrackbarMinMax::trackbarMaxCallbackStatic, this);
    }
}

TrackbarMinMax::TrackbarMinMax(std::string trackbar, std::string window, int *lo, int *hi, int range, bool showTrackbar)
{
    this->windowName = window;

    this->trackbarMinName = trackbar + " Min";

    this->trackbarMaxName = trackbar + " Max";

    this->minValue = lo;
    this->maxValue = hi;

    if (showTrackbar)
    {
        cv::namedWindow(this->windowName, cv::WINDOW_NORMAL);

        cv::createTrackbar(this->trackbarMinName, this->windowName, this->minValue, range, TrackbarMinMax::trackbarMinCallbackStatic, this);

        cv::createTrackbar(this->trackbarMaxName, this->windowName, this->maxValue, range, TrackbarMinMax::trackbarMaxCallbackStatic, this);
    }
}


void TrackbarMinMax::initTrackbar(std::string trackbar, std::string window, int &lo, int &hi, int range, bool showTrackbar)
{

    this->windowName = window;

    this->trackbarMinName = trackbar + " Min";

    this->trackbarMaxName = trackbar + " Max";

    this->minValue = &lo;
    this->maxValue = &hi;

    if (showTrackbar)
    {
        cv::namedWindow(this->windowName, cv::WINDOW_NORMAL);
        cv::createTrackbar(this->trackbarMinName, this->windowName, this->minValue, range, TrackbarMinMax::trackbarMinCallbackStatic, this);
        cv::createTrackbar(this->trackbarMaxName, this->windowName, this->maxValue, range, TrackbarMinMax::trackbarMaxCallbackStatic, this);
    }
}

void TrackbarMinMax::setValue(int minVal, int maxVal)
{
    *this->minValue = minVal;
    *this->maxValue = maxVal;

    if (showTrackbar)
    {
        cv::setTrackbarPos(this->trackbarMinName, this->windowName, minVal);
        cv::setTrackbarPos(this->trackbarMaxName, this->windowName, maxVal);
    }
}

void TrackbarMinMax::setMax(int value)
{
    *this->maxValue = value;

    if (showTrackbar)
        cv::setTrackbarPos(this->trackbarMaxName, this->windowName, value);
}

void TrackbarMinMax::setMin(int value)
{
    *this->minValue = value;

    if (showTrackbar)
        cv::setTrackbarPos(this->trackbarMinName, this->windowName, value);
}

int TrackbarMinMax::getMax()
{
    return *this->maxValue;
}

int TrackbarMinMax::getMin()
{
    return *this->minValue;
}

void TrackbarMinMax::trackbarMaxCallbackStatic(int value, void *userdata)
{
    TrackbarMinMax *handler = static_cast<TrackbarMinMax *>(userdata);
    // handler->min_value = value;
    handler->trackbarMaxCallback();
}

void TrackbarMinMax::trackbarMinCallbackStatic(int value, void *userdata)
{
    TrackbarMinMax *handler = static_cast<TrackbarMinMax *>(userdata);
    // handler->min_value = value;
    handler->trackbarMinCallback();
}

void TrackbarMinMax::trackbarMaxCallback()
{
    // IC(max_value);
    // if (*maxValue < *minValue + 1)
    //     cv::setTrackbarPos(trackbarMaxName, windowName, *minValue + 1);
}

void TrackbarMinMax::trackbarMinCallback()
{
    // IC(min_value);
    // if (*minValue > *maxValue - 1)
    //     cv::setTrackbarPos(trackbarMinName, windowName, *maxValue - 1);
}
TrackbarColour::TrackbarColour()
{
    Channel1 = new TrackbarMinMax();
    Channel2 = new TrackbarMinMax();
    Channel3 = new TrackbarMinMax();
}

TrackbarColour::TrackbarColour(std::string trackbar, std::string window, ColorChannel color_channel, int &Ch1_hi, int &Ch1_lo, int &Ch2_hi, int &Ch2_lo, int &Ch3_hi, int &Ch3_lo, int range, bool showTrackbar)
{
    IC(showTrackbar);
    std::string channel1, channel2, channel3;
    if (color_channel == ColorChannel::RGB)
    {
        channel1 = trackbar + " R";
        channel2 = trackbar + " G";
        channel3 = trackbar + " B";
    }
    else if (color_channel == ColorChannel::HSV)
    {
        channel1 = trackbar + " H";
        channel2 = trackbar + " S";
        channel3 = trackbar + " V";
    }
    else if (color_channel == ColorChannel::HLS)
    {
        channel1 = trackbar + " H";
        channel2 = trackbar + " L";
        channel3 = trackbar + " S";
    }
    else if (color_channel == ColorChannel::Luv)
    {
        channel1 = trackbar + " L";
        channel2 = trackbar + " U";
        channel3 = trackbar + " V";
    }
    else
    {
        channel1 = trackbar + " 1";
        channel2 = trackbar + " 2";
        channel3 = trackbar + " 3";
    }
    IC(channel1, channel2, channel3);
    Channel1 = new TrackbarMinMax(channel1, window, Ch1_lo, Ch1_hi, 255, showTrackbar);
    Channel2 = new TrackbarMinMax(channel2, window, Ch2_lo, Ch2_hi, 255, showTrackbar);
    Channel3 = new TrackbarMinMax(channel3, window, Ch3_lo, Ch3_hi, 255, showTrackbar);
}

TrackbarColour::TrackbarColour(std::string trackbar, std::string window, ColorChannel color_channel, int *Ch1_hi, int *Ch1_lo, int *Ch2_hi, int *Ch2_lo, int *Ch3_hi, int *Ch3_lo, int range, bool showTrackbar)
{
    IC(showTrackbar);
    std::string channel1, channel2, channel3;
    if (color_channel == ColorChannel::RGB)
    {
        channel1 = trackbar + " R";
        channel2 = trackbar + " G";
        channel3 = trackbar + " B";
    }
    else if (color_channel == ColorChannel::HSV)
    {
        channel1 = trackbar + " H";
        channel2 = trackbar + " S";
        channel3 = trackbar + " V";
    }
    else if (color_channel == ColorChannel::HLS)
    {
        channel1 = trackbar + " H";
        channel2 = trackbar + " L";
        channel3 = trackbar + " S";
    }
    else if (color_channel == ColorChannel::Luv)
    {
        channel1 = trackbar + " L";
        channel2 = trackbar + " U";
        channel3 = trackbar + " V";
    }
    else
    {
        channel1 = trackbar + " 1";
        channel2 = trackbar + " 2";
        channel3 = trackbar + " 3";
    }
    IC(channel1, channel2, channel3);
    Channel1 = new TrackbarMinMax(channel1, window, Ch1_lo, Ch1_hi, 255, showTrackbar);
    Channel2 = new TrackbarMinMax(channel2, window, Ch2_lo, Ch2_hi, 255, showTrackbar);
    Channel3 = new TrackbarMinMax(channel3, window, Ch3_lo, Ch3_hi, 255, showTrackbar);
}

void TrackbarColour::initTrackbarColour(std::string trackbar, std::string window, ColorChannel color_channel, int &Ch1_hi, int &Ch1_lo, int &Ch2_hi, int &Ch2_lo, int &Ch3_hi, int &Ch3_lo, int range, bool showTrackbar)
{
    IC();
    std::string channel1, channel2, channel3;
    if (color_channel == ColorChannel::RGB)
    {
        channel1 = trackbar + " R";
        channel1 = trackbar + " G";
        channel1 = trackbar + " B";
    }
    else if (color_channel == ColorChannel::HSV)
    {
        channel1 = trackbar + " H";
        channel1 = trackbar + " S";
        channel1 = trackbar + " V";
    }
    else if (color_channel == ColorChannel::HLS)
    {
        channel1 = trackbar + " H";
        channel1 = trackbar + " L";
        channel1 = trackbar + " S";
    }
    else if (color_channel == ColorChannel::Luv)
    {
        channel1 = trackbar + " L";
        channel1 = trackbar + " U";
        channel1 = trackbar + " V";
    }
    else
    {
        channel1 = trackbar + " 1";
        channel1 = trackbar + " 2";
        channel1 = trackbar + " 3";
    }
    int a, b;
    IC();
    Channel1->initTrackbar(channel1, window, Ch1_lo, Ch1_hi, 255, showTrackbar);
    IC();
    Channel2->initTrackbar(channel2, window, Ch2_lo, Ch2_hi, 255, showTrackbar);
    Channel3->initTrackbar(channel3, window, Ch3_lo, Ch3_hi, 255, showTrackbar);
}

void TrackbarColour::setLower(int ch1, int ch2, int ch3)
{
    Channel1->setMin(ch1);
    Channel2->setMin(ch2);
    Channel3->setMin(ch3);
}
void TrackbarColour::setUpper(int ch1, int ch2, int ch3)
{
    Channel1->setMax(ch1);
    Channel2->setMax(ch2);
    Channel3->setMax(ch3);
}

cv::Scalar TrackbarColour::getLower()
{
    return cv::Scalar(Channel1->getMin(),
                      Channel2->getMin(),
                      Channel3->getMin());
}

cv::Scalar TrackbarColour::getUpper()
{
    return cv::Scalar(Channel1->getMax(),
                      Channel2->getMax(),
                      Channel3->getMax());
}