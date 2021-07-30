# STM8L152-Discovery-I2C-test\
Sample code to use the I2C functionality on the ST STM8L152-Discovery board\
•PORTC0->SDA PORTC1->SCL\
•Both pins will be configured as open-drain outputs so pullup resistors must be placed (1k to 10k should work)\
•The solder bridge SB17 must be removed as it places 100nf on SCL which distorts the clock signal\
•The previous step disables the user pushbutton on the board\
•See stm8l15x_conf.h to check the peripheral header file inclusion needed.
