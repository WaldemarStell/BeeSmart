// Copyright (c) Microsoft. All rights reserved.
// Licensed under the MIT license.

// #include "HTS221Sensor.h"
// #include "LPS22HBSensor.h"
#include "Sensor.h"
#include "AzureIotHub.h"
#include "Arduino.h"
#include "parson.h"
#include "config.h"
#include "RGB_LED.h"
#include "utility.h"

#define RGB_LED_BRIGHTNESS 32
#define LOOP_DELAY 1000
#define EXPECTED_COUNT 5

DevI2C *i2c;
HTS221Sensor *sensor;

// The pressure sensor
LPS22HBSensor *lps22HBSensor;

// The magnetometer sensor
LIS2MDLSensor *lis2mdl;

static RGB_LED rgbLed;
static int interval = INTERVAL;
static float humidity;
static float temperature;
static float pressure;

// Data from magnetometer sensor
static int axes[3];
static int base_x;
static int base_y;
static int base_z;

// Indicate whether the magnetometer sensor has been initialized
static bool initialized = false;

// The open / close status of the door
static bool preOpened = false;

int getInterval()
{
    return interval;
}

void blinkLED()
{
    rgbLed.turnOff();
    rgbLed.setColor(RGB_LED_BRIGHTNESS, 0, 0);
    delay(500);
    rgbLed.turnOff();
}

void blinkSendConfirmation()
{
    rgbLed.turnOff();
    rgbLed.setColor(0, 0, RGB_LED_BRIGHTNESS);
    delay(500);
    rgbLed.turnOff();
}

void parseTwinMessage(DEVICE_TWIN_UPDATE_STATE updateState, const char *message)
{
    JSON_Value *root_value;
    root_value = json_parse_string(message);
    if (json_value_get_type(root_value) != JSONObject)
    {
        if (root_value != NULL)
        {
            json_value_free(root_value);
        }
        LogError("parse %s failed", message);
        return;
    }
    JSON_Object *root_object = json_value_get_object(root_value);

    double val = 0;
    if (updateState == DEVICE_TWIN_UPDATE_COMPLETE)
    {
        JSON_Object *desired_object = json_object_get_object(root_object, "desired");
        if (desired_object != NULL)
        {
            val = json_object_get_number(desired_object, "interval");
        }
    }
    else
    {
        val = json_object_get_number(root_object, "interval");
    }
    if (val > 500)
    {
        interval = (int)val;
        LogInfo(">>>Device twin updated: set interval to %d", interval);
    }
    json_value_free(root_value);
}

void SensorInit()
{
    i2c = new DevI2C(D14, D15);

    sensor = new HTS221Sensor(*i2c);
    sensor->init(NULL);

    lps22HBSensor = new LPS22HBSensor(*i2c);
    lps22HBSensor->init(NULL);

    lis2mdl = new LIS2MDLSensor(*i2c);
    lis2mdl->init(NULL);

    humidity = -1;
    temperature = -1000;
    pressure = -1;

    lis2mdl->getMAxes(axes);
    base_x = axes[0];
    base_y = axes[1];
    base_z = axes[2];

    int count = 0;
    int delta = 10;
    while (true)
    {
        delay(LOOP_DELAY);
        lis2mdl->getMAxes(axes);

        // Waiting for the data from sensor to become stable
        if (abs(base_x - axes[0]) < delta && abs(base_y - axes[1]) < delta && abs(base_z - axes[2]) < delta)
        {
            count++;
            if (count >= EXPECTED_COUNT)
            {
                break;
            }
        }
        else
        {
            count = 0;
            base_x = axes[0];
            base_y = axes[1];
            base_z = axes[2];
        }
    }
}

float readTemperature()
{
    sensor->reset();

    float temperature = 0;
    sensor->getTemperature(&temperature);

    return temperature;
}

float readHumidity()
{
    sensor->reset();

    float humidity = 0;
    sensor->getHumidity(&humidity);

    return humidity;
}

float readPressure()
{
    // lps22HBSensor->rese();

    float pressure = 0;
    lps22HBSensor->getPressure(&pressure);

    return pressure;
}

// void readMagnetometer()
// {
//     lis2mdl->getMAxes(axes);

//     base_x = axes[0];
//     base_y = axes[1];
//     base_z = axes[2];
// }

bool readMessage(int messageId, char *payload)
{
    JSON_Value *root_value = json_value_init_object();
    JSON_Object *root_object = json_value_get_object(root_value);
    char *serialized_string = NULL;

    json_object_set_number(root_object, "messageId", messageId);

    float t = readTemperature();
    float h = readHumidity();
    float p = readPressure();
    // readMagnetometer();

    bool temperatureAlert = false;
    temperature = t;
    json_object_set_number(root_object, "temperature", temperature);

    if (temperature > TEMPERATURE_ALERT_HIGHER || temperature < TEMPERATURE_ALERT_LOWER)
    {
        temperatureAlert = true;
    }
    humidity = h;
    json_object_set_number(root_object, "humidity", humidity);

    pressure = p;
    json_object_set_number(root_object, "pressure", pressure);
    lis2mdl->getMAxes(axes);

    json_object_set_number(root_object, "magnetometer(x)", axes[0]);
    json_object_set_number(root_object, "magnetometer(y)", axes[1]);
    json_object_set_number(root_object, "magnetometer(z)", axes[2]);

    int delta = 50;

    if (abs(base_x - axes[0]) < delta && abs(base_y - axes[1]) < delta && abs(base_z - axes[2]) < delta)
    {
        json_object_set_boolean(root_object, "Magnetsensor:Opened", false);
    }
    else
    {
        json_object_set_boolean(root_object, "Magnetsensor:Opened", true);
    }

    serialized_string = json_serialize_to_string_pretty(root_value);

    snprintf(payload, MESSAGE_MAX_LEN, "%s", serialized_string);
    json_free_serialized_string(serialized_string);
    json_value_free(root_value);
    return temperatureAlert;
}