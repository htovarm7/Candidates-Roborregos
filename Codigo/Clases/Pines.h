#ifndef Pines_h
#define Pines_h

// constexpr expresion constante
// unsigned integer of length 8 bits uint8_T


namespace Pines{
    constexpr uint8_t ultrasonicoFrontal[2] = {
        37, // Echo
        38  // Trigger
    };

    constexpr uint8_t ultrasonicoDerecha[2] = {
        52, // Echo
        53  // Trigger
    };

    constexpr uint8_t ultrasonicoIzquierda[2] = {
        22, // Echo
        24  // Trigger
    };  

    constexpr uint8_t motoresIzquierdos[6] = {
        5,  // ENA
        28, // INA1
        27, // INA2
        6,  // ENB
        26, // INB1
        25  // INB2
    };

    constexpr uint8_t motoresDerechos[6] = {
        7,  // ENA
        32, // INA1
        31, // INA2
        8,  // ENB
        30, // INB1
        29  // INB2
    };

    constexpr uint8_t ledRGB[3] = {
        9,  // R
        10, // B
        11  // G
    };

    constexpr uint8_t buzzer = 51;

    constexpr uint8_t motoresConEncoders[8] = {
        2,  // encoderSuperiorIzquierdo_A
        3,  // encoderSuperiorIzquierdo_B
        40, // encoderSuperiorDerecho_A
        41, // encoderSuperiorDerecho_B
        19, // encoderInferiorIzquierdo_A
        18, // encoderInferiorIzquierdo_B
        43, // encoderInferiorDerecho_A
        42  // encoderInferiorDerecho_B
    };

    constexpr uint8_t giroscopio[2] = {
        A5, // SCL
        A4  // SDA
    };

    constexpr uint8_t sensorColor[2] = {
        A4, // SDA
        A5  // SCL
    };

    constexpr uint8_t sensorLineaD1 = 43;
    constexpr uint8_t sensorLineaD2 = 42;
    constexpr uint8_t sensorLineaD3 = 41;
    constexpr uint8_t sensorLineaD4 = 40;
    constexpr uint8_t sensorLineaD5 = 39;
    constexpr uint8_t sensorLineaD6 = 38;
    constexpr uint8_t sensorLineaD7 = 37;
    constexpr uint8_t sensorLineaD8 = 36;

    constexpr uint8_t servo1 = 34;
    constexpr uint8_t servo2 = 35;

}

#endif /* Pines_h */