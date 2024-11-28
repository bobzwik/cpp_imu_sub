# cpp_imu_sub

## For LED control

```
git clone https://github.com/joan2937/pigpio
cd pigpio
make
sudo make install
```

And when running `listener_az_led`, run `sudo pigpiod` beforehand.

If running `listener_az_led2`, make sure `pigpiod` is killed with `sudo killall pigpiod`.
