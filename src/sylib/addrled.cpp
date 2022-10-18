#include "sylib/env.hpp"
#include "sylib/addrled.hpp"
#include "sylib/sylib_apitypes.hpp"
#include "sylib/system.hpp"



namespace sylib{
    Addrled::Addrled(const uint8_t smart_port, const uint8_t adi_port, const uint8_t strip_length, std::vector<uint32_t> colors) : 
                     Device(6,0), smart_port(smart_port), adi_port(adi_port), strip_length(strip_length), device(vexDeviceGetByIndex(smart_port-1)){
        
        std::unique_lock<sylib::Mutex> _lock;
        if (sylib::millis() > 1) {
            _lock = std::unique_lock<sylib::Mutex>(sylib_port_mutexes[smart_port-1]);
        }
        getAllAddrleds().push_back(this);        
        setAddrledUpdateCycles(getAllAddrleds().size());
        vexDeviceAdiPortConfigSet(device, adi_port-1, kAdiPortTypeDigitalOut);
        buffer.resize(strip_length);
        colors.resize(strip_length);
        for(int i = 0; i < strip_length; i++){
            buffer[i] = 0x000000;
        }
        for(int i = 0; i < colors.size(); i++){
            buffer[i] = colors[i];
        }
        addrledControlMode = SylibAddrledControlModeMANUAL;
    }
    Addrled::~Addrled(){
        getAllAddrleds().erase(std::remove(getAllAddrleds().begin(), getAllAddrleds().begin(), this));
    }
    std::uint32_t& Addrled::operator[] (std::uint32_t index){
        return buffer[index];
    }
    std::vector<uint32_t>& Addrled::operator*(){
        return buffer;
    }
    bool Addrled::addrled_enabled = true;
    const std::vector<uint32_t> Addrled::off_buffer = std::vector<uint32_t>(64,0x000000);
    std::vector<sylib::Addrled*>& Addrled::getAllAddrleds() {
        static auto allAddrleds = std::vector<sylib::Addrled *>();
        return allAddrleds;
    }
    void Addrled::setAddrledUpdateCycles(int count){
        int addrledCount = 0;
         for(auto & subTask : getAllAddrleds()){
            subTask->setUpdateFrequency(6*count);
            subTask->setUpdateOffset(6*addrledCount);
            addrledCount++;
        }
    }
    int Addrled::get_smart_port(){return smart_port;}
    int Addrled::get_adi_port(){return adi_port;}
    



    void Addrled::update(){
        mutex_lock _lock(sylib_port_mutexes[smart_port-1]);
        std::uint32_t currentTime = sylib::millis();
        static std::vector<uint32_t> preOverlayBuffer;
        static std::vector<uint32_t> lightOutput;
        lightOutput.resize(buffer.size());
        

        if(!addrled_enabled){
            vexDeviceAdiAddrLedSet(device, adi_port-1, (std::uint32_t*)off_buffer.data(), 0, 64, 0);
            return;
        }

        if(addrledControlMode == SylibAddrledControlModeOFF){
            vexDeviceAdiAddrLedSet(device, adi_port-1, (std::uint32_t*)off_buffer.data(), 0, 64, 0);
            return;
        }

        if(addrledControlMode == SylibAddrledControlModeMANUAL){
            lightOutput = buffer;
        }
        else if(addrledControlMode == SylibAddrledControlModeCYCLE){
            for(int i = 0; i < buffer.size(); i++){
                if(rotation_buffer[i] <= 0xFFFFFF){
                    buffer[i] = rotation_buffer[i];
                }
            }
            lightOutput = buffer;
            for(int i = 0; i < ((int)((currentTime - cycleStartMovementTime)/controlSpeed - cyclePixelsShifted)); i++){
                if(!cycleControlReversed){
                    std::rotate(rotation_buffer.begin(), rotation_buffer.end() - 1, rotation_buffer.end());
                }
                else{
                    std::rotate(rotation_buffer.begin(), rotation_buffer.begin() + 1, rotation_buffer.end());
                }
                cyclePixelsShifted++;             
            }
        }
        if(sendingPulse){
            preOverlayBuffer = buffer;
            for(int i = 0; i < buffer.size(); i++){
                if(template_buffer[i + (controlPulseWidth)] <= 0xFFFFFF){
                    buffer[i] = template_buffer[i + (controlPulseWidth)];
                }
            }
            
            lightOutput = buffer;
            for(int i = 0; i < ((int)((currentTime - pulseStartMovementTime)/pulseSpeed - pulsePixelsShifted)); i++){
                if(pulsePixelsShifted < (pixelsToMove-controlPulseWidth+1)){
                    if(!pulseControlReversed){
                        std::rotate(template_buffer.begin(), template_buffer.end() - 1, template_buffer.end());
                    }
                    else{
                        std::rotate(template_buffer.begin(), template_buffer.begin() + 1, template_buffer.end());
                    }
                    pulsePixelsShifted++;
                }
                else{
                    pulsePixelsShifted = 0;
                    sendingPulse = false;         
                    break;
                }
            }
            
            buffer = preOverlayBuffer;
        }
        preOverlayBuffer = buffer;
        for(int i = 0; i < buffer.size(); i++){
            rgb shiftedColor;
            shiftedColor.r = hex_to_rgb(lightOutput[i]).r + redShift;
            shiftedColor.g = hex_to_rgb(lightOutput[i]).g + greenShift;
            shiftedColor.b = hex_to_rgb(lightOutput[i]).b + blueShift;

            if(shiftedColor.r > 255){
                shiftedColor.r = 255;
            }
            else if(shiftedColor.r < 0){
                shiftedColor.r = 0;
            }

            if(shiftedColor.g > 255){
                shiftedColor.g = 255;
            }
            else if(shiftedColor.g < 0){
                shiftedColor.g = 0;
            }

            if(shiftedColor.b > 255){
                shiftedColor.b = 255;
            }
            else if(shiftedColor.b < 0){
                shiftedColor.b = 0;
            }

            buffer[i] = rgb_to_hex(shiftedColor.r, shiftedColor.g, shiftedColor.b);
        }
        lightOutput = buffer;
        buffer = preOverlayBuffer;

        vexDeviceAdiAddrLedSet(device, adi_port-1, (std::uint32_t*)lightOutput.data(), 0, strip_length, 0);
    }

    void Addrled::set_all(std::uint32_t color){
        mutex_lock _lock(sylib_port_mutexes[smart_port-1]);
        addrledControlMode = SylibAddrledControlModeMANUAL;
        for(int i = 0; i < buffer.size(); i++){
            buffer[i] = color;
        }
    }
    void Addrled::set_pixel(std::uint32_t color, std::uint8_t index){
        mutex_lock _lock(sylib_port_mutexes[smart_port-1]);
        addrledControlMode = SylibAddrledControlModeMANUAL;
        buffer[index] = color;
    }
    void Addrled::set_buffer(std::vector<std::uint32_t> colors){
        mutex_lock _lock(sylib_port_mutexes[smart_port-1]);
        addrledControlMode = SylibAddrledControlModeMANUAL;
        colors.resize(strip_length);
        buffer = colors;
    }
    void Addrled::turn_off(){
        mutex_lock _lock(sylib_port_mutexes[smart_port-1]);
        addrledControlMode = SylibAddrledControlModeOFF;
    }
    void Addrled::turn_on(){
        mutex_lock _lock(sylib_port_mutexes[smart_port-1]);
        addrledControlMode = SylibAddrledControlModeMANUAL;
    }
    void Addrled::clear(){
        mutex_lock _lock(sylib_port_mutexes[smart_port-1]);
        buffer.clear();
        buffer.resize(strip_length);
        addrledControlMode = SylibAddrledControlModeMANUAL;
    }
    void Addrled::pulse(std::uint32_t color, int pulse_width, int speed, int start_pos, bool reverse, int end_pixel){
        mutex_lock _lock(sylib_port_mutexes[smart_port-1]);
        sendingPulse = true;
        if(speed > 100){
            speed = 100;
        }
        else if(speed < 1){
            speed = 1;
        }
        pulseSpeed = 250.0/speed;
        controlPulseWidth = pulse_width;
        pulseControlReversed = reverse;
        template_buffer = std::vector<uint32_t>();
        if(end_pixel == -1){
            end_pixel = strip_length - 1;
        }
        pixelsToMove = end_pixel - start_pos + (pulse_width)*2;
        buffer_saved = buffer;
        template_buffer.resize(buffer.size() + 2*(pulse_width-1));
        for(int i = 0; i < buffer.size(); i++){
            template_buffer[i] = 0xFFFFFFFF;
        }
        for(int i = 0; i < pulse_width; i++){
            if(!reverse){
                template_buffer[i+start_pos] = color;
            }
            else{
                template_buffer[template_buffer.size() - 1 - i] = color;
            }
        }
        pulseStartMovementTime = sylib::millis();
    }
    std::uint32_t Addrled::interpolate_rgb(std::uint32_t start_color, std::uint32_t end_color, int step, int fade_width){
        rgb startComponents = hex_to_rgb(start_color);
        rgb endComponents = hex_to_rgb(end_color);

        double red_diff = endComponents.r - startComponents.r;
        double green_diff = endComponents.g - startComponents.g;
        double blue_diff = endComponents.b - startComponents.b;        

        double red_step = red_diff/fade_width;
        double green_step = green_diff/fade_width;
        double blue_step = blue_diff/fade_width;

        
        rgb solved;

        solved.r = (startComponents.r + red_step * step);
        solved.g = (startComponents.g + green_step * step);
        solved.b = (startComponents.b + blue_step * step);
        return rgb_to_hex(solved.r, solved.g, solved.b); 
    }
    std::uint32_t Addrled::interpolate_hsv(std::uint32_t start_color, std::uint32_t end_color, int step, int fade_width){
        hsv start_hsv = rgb_to_hsv(start_color);
        hsv end_hsv = rgb_to_hsv(end_color);

        double hue_diff = end_hsv.h - start_hsv.h;
        double sat_diff = end_hsv.s - start_hsv.s;
        double val_diff = end_hsv.v - start_hsv.v;

        double hue_step = hue_diff/fade_width;
        double sat_step = sat_diff/fade_width;
        double val_step = val_diff/fade_width;

        

        hsv interpolated;
        interpolated.h = (start_hsv.h + hue_step * step);
        interpolated.s = (start_hsv.s + sat_step * step);
        interpolated.v = (start_hsv.v + val_step * step);
        std::uint32_t solved = hsv_to_rgb(interpolated);

        return solved;
        
    }
    void Addrled::gradient(std::uint32_t start_color, std::uint32_t end_color, int fade_width, int start_pos, bool reverse, bool hsv_mode){
        mutex_lock _lock(sylib_port_mutexes[smart_port-1]);
        if(fade_width == 0){
            fade_width = strip_length;
        }
        for(int i = 0; i < fade_width; i++){
            if(hsv_mode){
                buffer[start_pos+i] = interpolate_hsv(start_color, end_color, i, fade_width);
            }
            else{
                buffer[start_pos+i] = interpolate_rgb(start_color, end_color, i, fade_width);
            }
        }
        addrledControlMode = SylibAddrledControlModeMANUAL;
    }
    void Addrled::cycle(std::vector<uint32_t> pattern, int speed, int start_pos, bool reverse){
        mutex_lock _lock(sylib_port_mutexes[smart_port-1]);
        addrledControlMode = SylibAddrledControlModeCYCLE;
        if(speed > 100){
            speed = 100;
        }
        else if(speed < 1){
            speed = 1;
        }
        controlSpeed = 250.0/speed;
        cycleStartMovementTime = sylib::millis();
        cyclePixelsShifted = 0;
        cycleControlReversed = reverse;
        rotation_buffer = pattern;
    }
    void Addrled::rotate(int pixels, bool reverse){
        mutex_lock _lock(sylib_port_mutexes[smart_port-1]);
        addrledControlMode = SylibAddrledControlModeMANUAL;
        if(!reverse){
            std::rotate(buffer.begin(), buffer.end() - pixels, buffer.end());
        }
        else{
            std::rotate(buffer.begin(), buffer.begin() + pixels, buffer.end());
        }
    }
    void Addrled::color_shift(int red, int green, int blue){
        mutex_lock _lock(sylib_port_mutexes[smart_port-1]);
        redShift = red;
        greenShift = green;
        blueShift = blue;
    }

    hsv Addrled::rgb_to_hsv(std::uint32_t color){
        rgb in;
        in.r = (color >> 16) & 0xff;
        in.g = (color >> 8) & 0xff;
        in.b = color  & 0xff; 
        hsv         out;
        double      min, max, delta;

        min = in.r < in.g ? in.r : in.g;
        min = min  < in.b ? min  : in.b;

        max = in.r > in.g ? in.r : in.g;
        max = max  > in.b ? max  : in.b;

        out.v = max;                                // v
        delta = max - min;
        if (delta < 0.00001)
        {
            out.s = 0;
            out.h = 0; // undefined, maybe nan?
            return out;
        }
        if( max > 0.0 ) { // NOTE: if Max is == 0, this divide would cause a crash
            out.s = (delta / max);                  // s
        } else {
            // if max is 0, then r = g = b = 0              
            // s = 0, h is undefined
            out.s = 0.0;
            out.h = NAN;                            // its now undefined
            return out;
        }
        if( in.r >= max ){                           // > is bogus, just keeps compilor happy
            out.h = ( in.g - in.b ) / delta;        // between yellow & magenta
        }
        else{
            if( in.g >= max ){
                out.h = 2.0 + ( in.b - in.r ) / delta;  // between cyan & yellow
            }
            else{
                out.h = 4.0 + ( in.r - in.g ) / delta;  // between magenta & cyan
            }
        }
        
        out.h = out.h*60.0;                              // degrees

        if( out.h < 0.0 )
            out.h += 360.0;

        out.v = out.v/255;

        return out;
    }

    std::uint32_t Addrled::rgb_to_hex(int r, int g, int b){
        return (((r & 0xff) << 16) + ((g & 0xff) << 8) + (b & 0xff));
    }
    std::uint32_t Addrled::hsv_to_rgb(hsv in){
        double      hh, p, q, t, ff;
        long        i;
        rgb         out;
        // in.v *= 255;
        if(in.s <= 0.0) {       // < is bogus, just shuts up warnings
            out.r = in.v;
            out.g = in.v;
            out.b = in.v;
            return (int)std::round(out.r*0xFF0000 + out.g*0xFF00 + out.b*0xFF);     
        }
        hh = in.h;
        if(hh >= 360.0) hh = 0.0;
        hh /= 60.0;
        i = (long)hh;
        ff = hh - i;
        p = in.v * (1.0 - in.s);
        q = in.v * (1.0 - (in.s * ff));
        t = in.v * (1.0 - (in.s * (1.0 - ff)));

        switch(i) {
        case 0:
            out.r = in.v;
            out.g = t;
            out.b = p;
            break;
        case 1:
            out.r = q;
            out.g = in.v;
            out.b = p;
            break;
        case 2:
            out.r = p;
            out.g = in.v;
            out.b = t;
            break;

        case 3:
            out.r = p;
            out.g = q;
            out.b = in.v;
            break;
        case 4:
            out.r = t;
            out.g = p;
            out.b = in.v;
            break;
        case 5:
        default:
            out.r = in.v;
            out.g = p;
            out.b = q;
            break;
        }
        return rgb_to_hex(out.r*0xFF, out.g*0xFF, out.b*0xFF);
    }
    rgb Addrled::hex_to_rgb(std::uint32_t color){
        rgb in;
        in.r = (color >> 16) & 0xff;
        in.g = (color >> 8) & 0xff;
        in.b = color  & 0xff; 
        return in;
    }
}