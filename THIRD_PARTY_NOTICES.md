# Third-party DMP component

The included MotionApps 6.12 firmware image and the basis for the MPU6050 DMP
initialization sequence come from Jeff Rowberg's i2cdevlib project:

- Repository: https://github.com/jrowberg/i2cdevlib
- Pinned revision: `8eda2ce663c05c7b490778fadf1a1f5848e1b268`
- Source: `Arduino/MPU6050/MPU6050_6Axis_MotionApps612.cpp`
- Source URL: https://github.com/jrowberg/i2cdevlib/blob/8eda2ce663c05c7b490778fadf1a1f5848e1b268/Arduino/MPU6050/MPU6050_6Axis_MotionApps612.cpp
- Embedded image: 3062 bytes, copied without changes.
- Image SHA256: `ae42d0be801847e53e77a96ca74961d3063a21f1c0006c556b13d0e305a3c1ec`
- Packet format: MotionApps 6.12, 28 bytes, quaternion in the first 16 bytes.

The Mbed transport, FIFO polling, error handling, backend interface, and flight
axis adapter are implemented locally. Arduino, Wire, and the rest of i2cdevlib
are not build dependencies. The upstream source describes the image as a capture
of InvenSense MotionApps firmware; no claim is made to ownership of that firmware.
The license notice carried by the upstream file is retained below and in
`src/mpu6050_dmp_firmware.cpp`.

```text
/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/
```

The HC-SR04 and original MPU6050/Madgwick notices described in README.md continue
to apply to their respective components.
