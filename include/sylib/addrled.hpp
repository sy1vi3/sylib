/**
 * \file include/sylib/addrled.hpp
 *
 * \brief Contains prototypes for functions relating to individually
 * addressable LED strips for the V5
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#pragma once
#include <cstdint>
#include <vector>

#include "sylib/env.hpp"
#include "sylib/system.hpp"
#include "sylib/sylib_apitypes.hpp"


namespace sylib {

/**
 * \brief RGB Color
 */
typedef struct {
    double r;
    double g;
    double b;
} rgb;

/**
 * \brief HSV Color
 */
typedef struct {
    double h;
    double s;
    double v;
} hsv;

/**
 * \brief WS2812B Addressable LED Strip Controller
 */
class Addrled : private Device {
   private:
    static const std::vector<uint32_t> off_buffer;
    const std::uint8_t smart_port;
    const std::uint8_t adi_port;
    const std::uint8_t strip_length;
    std::vector<uint32_t> buffer;
    std::vector<uint32_t> template_buffer;
    std::vector<uint32_t> rotation_buffer;
    std::vector<uint32_t> buffer_saved;
    static std::vector<sylib::Addrled*>& getAllAddrleds();
    const V5_DeviceT device;
    static void setAddrledUpdateCycles(int count);
    sylib::SylibAddrledControlMode addrledControlMode;
    double controlSpeed        = 1;
    double pulseSpeed          = 1;
    int cyclePixelsShifted     = 0;
    int end_pixel              = -1;
    bool pulseControlReversed  = false;
    bool cycleControlReversed  = false;
    int pixelsToMove           = 0;
    int pulseStartMovementTime = 0;
    int cycleStartMovementTime = 0;
    bool sendingPulse          = false;
    int controlPulseWidth      = 1;
    int redShift               = 0;
    int greenShift             = 0;
    int blueShift              = 0;
    int pulsePixelsShifted     = 0;

   public:
    /**
     * \brief Creates an Addrled object for the given port and specifications.
     *
     * \param smart_port
     *        The V5 port number from 1-22
     * \param adi_port
     *        The ADI port number from 1-8
     * \param strip_length
     *        The number of pixels on the light strip
     * \param colors
     *        Vector of hex codes to initialize the strip to
     */
    Addrled(const std::uint8_t smart_port, const std::uint8_t adi_port,
            const std::uint8_t strip_length,
            const std::vector<std::uint32_t> colors = std::vector<std::uint32_t>());

    ~Addrled();

    /**
     * \brief A flag that when set to false turns off all lights on all addrled objects
     **/
    static bool addrled_enabled;

    /**
     * \brief Sets every pixel to a single color
     *
     * \param color
     *        Hex color code
     */
    void set_all(std::uint32_t color);

    /**
     * \brief Sets a specific pixel to a specific color
     *
     * \param color
     *        Hex color code
     * \param index
     *        Zero-indexed pixel number
     */
    void set_pixel(std::uint32_t color, std::uint8_t index);

    /**
     * \brief Sets the entire light strip to the provided vector of hex codes
     *
     * \param colors
     *        A buffer containing hex codes
     */
    void set_buffer(std::vector<uint32_t> colors);

    /**
     * \brief Addrled update loop
     *
     * Updates the light strip to the most recently set values
     *
     * Users do not need to call this function, it is handled
     * by the sylib daemon. Don't use this. It won't help.
     */
    void update() override;

    /**
     * \brief Sets the light control mode to OFF without changing the saved buffer
     */
    void turn_off();

    /**
     * \brief Sets the light control mode to MANUAL without changing the saved buffer
     */
    void turn_on();

    /**
     * \brief Resets the buffer
     *
     * Turns off the lights
     */
    void clear();

    /**
     * \brief Sends a single pulse of color down the strip
     *
     * \param color
     *        Hex code for the desired pulse color
     *
     * \param pulse_width
     *        The number of pixels wide the pulse should be
     *
     * \param speed
     *        A value from 1-100 describing the speed of the pulse. 1 is slow, 100 is fast
     *
     * \param start_pos
     *        How many pixels down the strip to start
     *
     * \param reverse
     *        A flag that when true reverses the direction of the pulse
     *
     * \param end_pixel
     *        The pixel number to end the pulse on. If this unset it defaults to the end of the
     * strip
     */
    void pulse(std::uint32_t color, int pulse_width, int speed = 1, int start_pos = 0,
               bool reverse = false, int end_pixel = -1);

    /**
     * \brief Sets the strip to a static color gradient
     *
     * Does not modify the saved buffer, can be used over other control modes
     *
     * \param start_color
     *        Hex code for one endpoint of the gradient
     *
     * \param end_color
     *        Hex code for the other endpoint of the gradient
     *
     * \param fade_width
     *        A value describing how many pixels long to make the transition between colors.
     *
     * \param reverse
     *        How many pixels down the strip to start
     *
     * \param reverse
     *        A flag that when true reverses the direction of the gradient
     *
     * \param hsv_mode
     *        A flag that enables the gradient to work in HSV space rather than RGB
     */
    void gradient(std::uint32_t start_color, std::uint32_t end_color, int fade_width = 0,
                  int start_pos = 0, bool reverse = false, bool hsv_mode = false);

    /**
     * \brief Automatically rotates the colors of a provided buffer through the light strip
     * periodically
     *
     * Sets the control mode to CYCLE
     *
     * \param color
     *        Buffer to rotate, use *<Addrled object name> to use the current saved buffer
     *
     * \param speed
     *        A value from 1-100 describing the speed of the pulse. 1 is slow, 100 is fast
     *
     * \param start_pos
     *        How many pixels down the strip to start
     *
     * \param reverse
     *        A flag that when true reverses the direction of the rotation
     */
    void cycle(std::vector<uint32_t> pattern, int speed = 1, int start_pos = 0,
               bool reverse = false);

    /**
     * \brief Rotates pixel colors up or down the strip
     *
     * \param pixels
     *        The number of pixels to augment the buffer by
     *
     * \param reverse
     *        A flag that when true reverses the direction of the rotation
     */
    void rotate(int pixels = 0, bool reverse = false);

    /**
     * \brief Adds the provided red, green, and blue values to the colors sent to the lights.
     *
     * Does not modify the saved buffer, can be used over other control modes
     *
     * \param red
     *        A number from -255 to 255 describing how much red to add
     *
     * \param green
     *        A number from -255 to 255 describing how much green to add
     *
     * \param blue
     *        A number from -255 to 255 describing how much blue to add
     */
    void color_shift(int red, int green, int blue);

    /**
     * \brief Gets smart port number used by the ADI expander for the light strip
     *
     * \return Value from 1-22 describing a smart port. 22 is the built-in three wire ports.
     */
    int get_smart_port();

    /**
     * \brief Gets adi port number used by the light strip
     *
     * \return Value from 1-8 describing an adi port.
     */
    int get_adi_port();

    /**
     * \brief Operator overload to retrieve a pixel color
     *
     * \param index
     *        Zero-indexed pixel number
     *
     * \return Hex code describing pixel color
     */
    uint32_t& operator[](std::uint32_t index);

    /**
     * \brief Operator overload to retrieve the current saved buffer
     *
     * \return Vector of hex codes describing pixel colors
     */
    std::vector<uint32_t>& operator*();

    /**
     * \brief Gets which control mode the light is using
     *
     * \return Control mode
     */
    sylib::SylibAddrledControlMode getControlMode();

    /**
     * \brief Finds a color between two provided color values
     *
     * Uses the HSV color space
     *
     * \param start_color
     *        The color at the start of the gradient
     *
     * \param end_color
     *        The color at the end of the gradient
     *
     * \param step
     *        The number of steps of size (1/fade_width) from the start value towards the end value
     *
     * \param fade_width
     *        The number of pixels between the two endpoints
     *
     * \return Hex code describing pixel color
     */
    static std::uint32_t interpolate_hsv(std::uint32_t start_color, std::uint32_t end_color,
                                         int step, int fade_width);

    /**
     * \brief Finds a color between two provided color values
     *
     * Uses the RGB color space
     *
     * \param start_color
     *        The color at the start of the gradient
     *
     * \param end_color
     *        The color at the end of the gradient
     *
     * \param step
     *        The number of steps of size (1/fade_width) from the start value towards the end value
     *
     * \param fade_width
     *        The number of pixels between the two endpoints
     *
     * \return Hex code describing pixel color
     */
    static std::uint32_t interpolate_rgb(std::uint32_t start_color, std::uint32_t end_color,
                                         int step, int fade_width);

    /**
     * \brief Converts an HSV color to an RGB hex code
     *
     * \param in
     *        HSV color to translate
     *
     * \return Hex code describing an RGB color
     */
    static std::uint32_t hsv_to_rgb(hsv in);

    /**
     * \brief Converts RGB values to an HSV color
     *
     * \param r
     *        Red amount
     *
     * \param g
     *        Green amount
     *
     * \param b
     *        Blue amount
     *
     * \return HSV color
     */
    static hsv rgb_to_hsv(std::uint32_t in);

    /**
     * \brief Converts RGB values to an RGB hex code
     *
     * \param r
     *        Red amount
     *
     * \param g
     *        Green amount
     *
     * \param b
     *        Blue amount
     *
     * \return Hex code describing an RGB color
     */
    static std::uint32_t rgb_to_hex(int r, int g, int b);

    /**
     * \brief Converts an RGB hex code to RGB values
     *
     * \param r
     *        Red amount
     *
     * \param g
     *        Green amount
     *
     * \param b
     *        Blue amount
     *
     * \return RGB color
     */
    static rgb hex_to_rgb(std::uint32_t color);
};
}  // namespace sylib