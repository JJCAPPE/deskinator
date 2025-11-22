- Make these changes to the software setup

# Hardware I2C bus 1 @ 100 kHz
dtparam=i2c_arm=on,i2c_arm_baudrate=100000

# Software I2C buses (slow them down a bit)
dtoverlay=i2c-gpio,bus=3,i2c_gpio_sda=15,i2c_gpio_scl=14,i2c_gpio_delay_us=5
dtoverlay=i2c-gpio,bus=5,i2c_gpio_sda=6,i2c_gpio_scl=13,i2c_gpio_delay_us=5
dtoverlay=i2c-gpio,bus=7,i2c_gpio_sda=19,i2c_gpio_scl=26,i2c_gpio_delay_us=5

- Switch right sensor from gpio 14-15 to 24-25

- Shorten wires

- Enable pullups
gpio=14=pu

- Align the proximity driver to the stable gesture settings (PPULSE) and optionally enable LED boost. This increases emitted IR energy and helps raw returns rise out of the 4–10 range.



