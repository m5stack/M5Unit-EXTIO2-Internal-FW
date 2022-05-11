## M5Unit-EXTIO2-Internal-FW

### Register

- I2C ADDR: 0x45

![protocol_01](/images/protocol_01.jpg)

>MODE CONFING

| REG  | DESC      | LEN    | R/W |
| ---- | --------- | ------ | --- |
| 0x00 | MODE_CH_1 | 1 BYTE | R/W |
| 0x01 | MODE_CH_2 | 1 BYTE | R/W |
| 0x02 | MODE_CH_3 | 1 BYTE | R/W |
| 0x03 | MODE_CH_4 | 1 BYTE | R/W |
| 0x04 | MODE_CH_5 | 1 BYTE | R/W |
| 0x05 | MODE_CH_6 | 1 BYTE | R/W |
| 0x06 | MODE_CH_7 | 1 BYTE | R/W |
| 0x07 | MODE_CH_8 | 1 BYTE | R/W |

- Value:

```
DIGITAL_INPUT_MODE=0 
DIGITAL_OUTPUT_MODE=1 
ADC_INPUT_MODE=2
SERVO_CTL_MODE=3 
RGB_LED_MODE=4
```

>DIGITAL INPUT/OUTPUT

| REG  | DESC                   | LEN    | R/W |
| ---- | ---------------------- | ------ | --- |
| 0x10 | OUTPUT_CTL_REG_CH_1    | 1 BYTE | R/W |
| 0x11 | OUTPUT_CTL_REG_CH_2    | 1 BYTE | R/W |
| 0x12 | OUTPUT_CTL_REG_CH_3    | 1 BYTE | R/W |
| 0x13 | OUTPUT_CTL_REG_CH_4    | 1 BYTE | R/W |
| 0x14 | OUTPUT_CTL_REG_CH_5    | 1 BYTE | R/W |
| 0x15 | OUTPUT_CTL_REG_CH_6    | 1 BYTE | R/W |
| 0x16 | OUTPUT_CTL_REG_CH_7    | 1 BYTE | R/W |
| 0x17 | OUTPUT_CTL_REG_CH_8    | 1 BYTE | R/W |
| 0x20 | DIGITAL_INPUT_REG_CH_1 | 1 BYTE | R   |
| 0x21 | DIGITAL_INPUT_REG_CH_2 | 1 BYTE | R   |
| 0x22 | DIGITAL_INPUT_REG_CH_3 | 1 BYTE | R   |
| 0x23 | DIGITAL_INPUT_REG_CH_4 | 1 BYTE | R   |
| 0x24 | DIGITAL_INPUT_REG_CH_5 | 1 BYTE | R   |
| 0x25 | DIGITAL_INPUT_REG_CH_6 | 1 BYTE | R   |
| 0x26 | DIGITAL_INPUT_REG_CH_7 | 1 BYTE | R   |
| 0x27 | DIGITAL_INPUT_REG_CH_8 | 1 BYTE | R   |

- Value:

```
HIGH:1 / LOW:0
```

>8B ANALOG INPUT

| REG  | DESC                                     | LEN    | R/W |
| ---- | ---------------------------------------- | ------ | --- |
| 0x30 | ANALOG_INPUT_8B_REG_CH_1<br>Value: 0-255 | 1 BYTE | R   |
| 0x31 | ANALOG_INPUT_8B_REG_CH_2<br>Value: 0-255 | 1 BYTE | R   |
| 0x32 | ANALOG_INPUT_8B_REG_CH_3<br>Value: 0-255 | 1 BYTE | R   |
| 0x33 | ANALOG_INPUT_8B_REG_CH_4<br>Value: 0-255 | 1 BYTE | R   |
| 0x34 | ANALOG_INPUT_8B_REG_CH_5<br>Value: 0-255 | 1 BYTE | R   |
| 0x35 | ANALOG_INPUT_8B_REG_CH_6<br>Value: 0-255 | 1 BYTE | R   |
| 0x36 | ANALOG_INPUT_8B_REG_CH_7<br>Value: 0-255 | 1 BYTE | R   |
| 0x37 | ANALOG_INPUT_8B_REG_CH_8<br>Value: 0-255 | 1 BYTE | R   |

>12B ANALOG INPUT

| REG  | DESC                                       | LEN    | R/W |
| ---- | ------------------------------------------ | ------ | --- |
| 0x40 | ANALOG_INPUT_12B_REG_CH_1<br>Value: 0-4095 | 2 BYTE | R   |
| 0x42 | ANALOG_INPUT_12B_REG_CH_2<br>Value: 0-4095 | 2 BYTE | R   |
| 0x44 | ANALOG_INPUT_12B_REG_CH_3<br>Value: 0-4095 | 2 BYTE | R   |
| 0x46 | ANALOG_INPUT_12B_REG_CH_4<br>Value: 0-4095 | 2 BYTE | R   |
| 0x48 | ANALOG_INPUT_12B_REG_CH_5<br>Value: 0-4095 | 2 BYTE | R   |
| 0x4A | ANALOG_INPUT_12B_REG_CH_6<br>Value: 0-4095 | 2 BYTE | R   |
| 0x4C | ANALOG_INPUT_12B_REG_CH_7<br>Value: 0-4095 | 2 BYTE | R   |
| 0x4E | ANALOG_INPUT_12B_REG_CH_8<br>Value: 0-4095 | 2 BYTE | R   |

>SERVO ANGLE CTL

| REG  | DESC                                       | LEN    | R/W |
| ---- | ------------------------------------------ | ------ | --- |
| 0x50 | SERVO_ANGLE_8B_REG_CH_1<br>Value: 0-180deg | 1 BYTE | R/W |
| 0x51 | SERVO_ANGLE_8B_REG_CH_2<br>Value: 0-180deg | 1 BYTE | R/W |
| 0x52 | SERVO_ANGLE_8B_REG_CH_3<br>Value: 0-180deg | 1 BYTE | R/W |
| 0x53 | SERVO_ANGLE_8B_REG_CH_4<br>Value: 0-180deg | 1 BYTE | R/W |
| 0x54 | SERVO_ANGLE_8B_REG_CH_5<br>Value: 0-180deg | 1 BYTE | R/W |
| 0x55 | SERVO_ANGLE_8B_REG_CH_6<br>Value: 0-180deg | 1 BYTE | R/W |
| 0x56 | SERVO_ANGLE_8B_REG_CH_7<br>Value: 0-180deg | 1 BYTE | R/W |
| 0x57 | SERVO_ANGLE_8B_REG_CH_8<br>Value: 0-180deg | 1 BYTE | R/W |

>SERVO PULSE CTL

| REG  | DESC                                         | LEN    | R/W |
| ---- | -------------------------------------------- | ------ | --- |
| 0x60 | SERVO_PULSE_16B_REG_CH_1<br> Value: 0-2500us | 2 BYTE | R/W |
| 0x62 | SERVO_PULSE_16B_REG_CH_2<br> Value: 0-2500us | 2 BYTE | R/W |
| 0x64 | SERVO_PULSE_16B_REG_CH_3<br> Value: 0-2500us | 2 BYTE | R/W |
| 0x66 | SERVO_PULSE_16B_REG_CH_4<br> Value: 0-2500us | 2 BYTE | R/W |
| 0x68 | SERVO_PULSE_16B_REG_CH_5<br> Value: 0-2500us | 2 BYTE | R/W |
| 0x6A | SERVO_PULSE_16B_REG_CH_6<br> Value: 0-2500us | 2 BYTE | R/W |
| 0x6C | SERVO_PULSE_16B_REG_CH_7<br> Value: 0-2500us | 2 BYTE | R/W |
| 0x6E | SERVO_PULSE_16B_REG_CH_8<br> Value: 0-2500us | 2 BYTE | R/W |

>RGB LED CTL

| REG  | DESC                      | LEN    | R/W |
| ---- | ------------------------- | ------ | --- |
| 0x70 | RGB_24B_REG_CH_1：RGB 888 | 3 BYTE | R/W |
| 0x73 | RGB_24B_REG_CH_2：RGB 888 | 3 BYTE | R/W |
| 0x76 | RGB_24B_REG_CH_3：RGB 888 | 3 BYTE | R/W |
| 0x79 | RGB_24B_REG_CH_4：RGB 888 | 3 BYTE | R/W |
| 0x7C | RGB_24B_REG_CH_5：RGB 888 | 3 BYTE | R/W |
| 0x7F | RGB_24B_REG_CH_6：RGB 888 | 3 BYTE | R/W |
| 0x82 | RGB_24B_REG_CH_7：RGB 888 | 3 BYTE | R/W |
| 0x85 | RGB_24B_REG_CH_8：RGB 888 | 3 BYTE | R/W |

>CONFIG

| REG  | DESC                                                                | LEN    | R/W |
| ---- | ------------------------------------------------------------------- | ------ | --- |
| 0xFE | FW VERSION                                                          | 1 BYTE | R   |
| 0xFF | I2C ADDR CONFIG (warn: Repeated writing may cause partition damage) | 1 BYTE | R/W |
