
# Sylib 

 
Sylvie's C++ Library For V5

## Features

- Easy WS2812B Addressable LED control
- Accurate motor velocity measurements
- Template for making custom velocity controllers
- Many kinds of pre-built filters for general use
- Platform-agnostic, meaning it works with both PROS and VEXcode


## Installation

### PROS

1) Download the latest version of the Sylib template from the [Releases] page

2) In the directory where you downloaded the zip archive, run `pros c fetch sylib@<version>.zip`

3) In your PROS project directory, run `pros c apply sylib@<version>`

4) In your `main.h` file, make sure to include `sylib/sylib.hpp`


### VEXcode/VEX Visual Studio Code Extension

1) From the [Releases] page, download the latest version of the Sylib source code 

2) Copy the contents of the `include/` directory from the downloaded files into the `include/` directory of your project

3) Copy the contents of the `src/` directory from the downloaded files into the `src/` directory of your project

4) Include `sylib/sylib.hpp` anywhere relevant, probably including at the top of your `main.cpp` file

## Docs

Documentation for Sylib can be found [here]