# Third-party notice

`epd4g` contains data and a register sequence derived from the **Waveshare e-Paper** library
(https://github.com/waveshareteam/e-Paper, file `EPD_2in7_V2.c`, author: Waveshare team):

* the stock 4-gray LUT table `lut_stock[]` in `src/epd4g.c`;
* the controller register sequence of `epd4g_init()` (the stock `EPD_2IN7_V2_Init_4GRAY`);
* the RAM bit encoding of the 4-gray image (verified byte for byte by `tests/`).

The Waveshare file carries the following permission notice, which applies to those parts:

> Permission is hereby granted, free of charge, to any person obtaining a copy
> of this software and associated documnetation files (the "Software"), to deal
> in the Software without restriction, including without limitation the rights
> to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
> copies of the Software, and to permit persons to  whom the Software is
> furished to do so, subject to the following conditions:
>
> The above copyright notice and this permission notice shall be included in
> all copies or substantial portions of the Software.
>
> THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
> IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
> FITNESS OR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
> AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
> LIABILITY WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
> OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
> THE SOFTWARE.

If you copy `src/epd4g.c` into another project, keep this notice with it.
