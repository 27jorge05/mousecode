#include "fusionangular.h"

static const uint8_t MPU6050_REG_PWR_MGMT_1 = 0x6B;
static const uint8_t MPU6050_REG_GYRO_CONFIG = 0x1B;
static const uint8_t MPU6050_REG_GYRO_ZOUT_H = 0x47;
static const uint8_t MPU6050_REG_WHO_AM_I = 0x75;

// FS_SEL = 3  →  ±2000 °/s  →  16.4 LSB/(°/s)
static const float GYRO_LSB_PER_DPS = 16.4f;
static const float DEG2RAD = PI / 180.0f;

FusionAngular::FusionAngular(uint8_t sda, uint8_t scl, uint8_t addr, float alpha)
    : _wire(&Wire), _sda(sda), _scl(scl), _addr(addr), _alpha(alpha),
      _biasZ(0.0f), _omegaZ(0.0f), _phiIMU(0.0f), _phiFusion(0.0f), _ok(false) {}

bool FusionAngular::begin()
{
    // Pull-ups internos débiles como red de seguridad (los módulos MPU6050
    // normalmente traen 4.7 kΩ externos, que son los que mandan).
    pinMode(_sda, INPUT_PULLUP);
    pinMode(_scl, INPUT_PULLUP);
    bool i2cIniciado = Wire.begin(
        _sda,
        _scl,
        400000);

    if (!i2cIniciado)
    {
        Serial.println("ERROR: no se pudo iniciar el bus I2C.");
        return 0;
    }
    else
        Serial.println("Se pudo iniciar el bus I2C.");

    uint8_t who = _leerReg(MPU6050_REG_WHO_AM_I);
    if (who != 0x68 && who != 0x69)
    {
        Serial.println(" devolucion de mpu6050 (valor de who)");
        Serial.println(who);
        _ok = false;
        return false;
    }

    _escribirReg(MPU6050_REG_PWR_MGMT_1, 0x00); // despertar
    delay(50);
    _escribirReg(MPU6050_REG_GYRO_CONFIG, 0x18); // FS_SEL=3 → ±2000 °/s
    delay(10);

    _ok = true;
    return true;
}

void FusionAngular::calibrar(uint16_t muestras)
{
    if (!_ok)
        return;

    float suma = 0.0f;
    uint16_t n = 0;
    for (uint16_t i = 0; i < muestras; i++)
    {
        float gz;
        if (_leerGyroZ(gz))
        {
            suma += gz;
            n++;
        }
        delayMicroseconds(500);
    }
    if (n > 0)
        _biasZ = suma / n;
    else
        _ok = false;
    _omegaZ = 0.0f;
}

void FusionAngular::reiniciar(float phi_inicial)
{
    _phiIMU = phi_inicial;
    _phiFusion = phi_inicial;
    _omegaZ = 0.0f;
}

void FusionAngular::actualizar(float dt, float phi_enc)
{
    if (!_ok)
        return;

    float gz;
    if (!_leerGyroZ(gz))
    {
        _ok = false;
        return;
    }

    _omegaZ = gz;                     // rad/s crudo
    float dGyro = (gz - _biasZ) * dt; // radianes según gyro

    _phiIMU += dGyro;

    // Filtro complementario
    _phiFusion = _alpha * (_phiFusion + dGyro) + (1.0f - _alpha) * phi_enc;
}

uint8_t FusionAngular::_leerReg(uint8_t reg)
{

    _wire->beginTransmission(0x68);
    uint8_t resultado68 = _wire->endTransmission(true);

    _wire->beginTransmission(0x69);
    uint8_t resultado69 = _wire->endTransmission(true);

    Serial.print("Respuesta direccion 0x68: ");
    Serial.println(resultado68);

    Serial.print("Respuesta direccion 0x69: ");
    Serial.println(resultado69);

    Serial.print("Nivel SDA: ");
    Serial.println(digitalRead(_sda));

    Serial.print("Nivel SCL: ");
    Serial.println(digitalRead(_scl));

    
    _wire->beginTransmission(_addr);
    _wire->write(reg);
    if (_wire->endTransmission(false) != 0)
        return 0;
    size_t bytesRecibidos = _wire->requestFrom(_addr, (uint8_t)1);

    Serial.print("Bytes recibidos: ");
    Serial.println(bytesRecibidos);

    if (bytesRecibidos != 1)
        return 0;
    return _wire->read();
}

void FusionAngular::_escribirReg(uint8_t reg, uint8_t val)
{
    _wire->beginTransmission(_addr);
    _wire->write(reg);
    _wire->write(val);
    _wire->endTransmission();
}

bool FusionAngular::_leerGyroZ(float &gz_rads)
{
    _wire->beginTransmission(_addr);
    _wire->write(MPU6050_REG_GYRO_ZOUT_H);
    if (_wire->endTransmission(false) != 0)
        return false;
    if (_wire->requestFrom(_addr, (uint8_t)2) != 2)
        return false;

    int16_t raw = (int16_t)((_wire->read() << 8) | _wire->read());
    float dps = raw / GYRO_LSB_PER_DPS;
    gz_rads = dps * DEG2RAD;
    return true;
}