#include <Arduino.h>
#include <M5StickCPlus.h>
#include <BluetoothSerial.h>
#include <ELMduino.h>
#include <EEPROM.h>

#include "AppUI.h"
#include "AppStatus.h"
#include "AppSettings.h"
#include "MutualM5.h"

// -------- Config: nome e PIN do dispositivo Bluetooth alvo --------
const char* targetBtName = "OBDII";   // <-- ajuste para o NOME exato anunciado pelo seu dongle (ex.: "V-LINK", "ELM327", "CHX")
const char* targetBtPin  = "1234";    // PIN comum: "1234" ou "0000"

// -----------------------------------------------------------------
AppSettings appSettings;
AppStatus appStatus;
AppUI appUI = AppUI(&appStatus, &appSettings);

BluetoothSerial SerialBT;
ELM327 elm327;
// (opcional) fallback por MAC, se quiser manter:
uint8_t elm327MacAddress[6] = {0x8D, 0x04, 0xF8, 0x00, 0x52, 0x2F};

uint64_t chipid;
char chipname[256];

// ---------------- Tasks ----------------
void heartBeatTask(void *arg)
{  
  while (1)
  {
    appStatus.lock();

    appStatus.heartBeat = !appStatus.heartBeat;

    bool isOperating = appStatus.bluetoothConnected && appStatus.elm327Connected;
    // ELMduino 3.x: usar nb_rx_state no lugar de status
    bool isOperatingOk = isOperating && appStatus.lastElm327Status == ELM_SUCCESS;

    appStatus.unlock();

    delay(isOperatingOk ? 250 : 1000);
  }
}

// ----------- Nova função: conectar por NOME -----------
bool connectBluetoothByName(const char* remoteName,
                            uint32_t retryDelayMs = 300,
                            uint32_t timeoutMs    = 15000)
{
  appStatus.mutualUpdate([](AppStatus* s){
    s->bluetoothConnected = false;
    s->elm327Connected    = false;
  });

  uint32_t t0 = millis();
  while ((millis() - t0) < timeoutMs) {
    if (SerialBT.connect(remoteName)) {
      appStatus.mutualUpdate([](AppStatus* s){
        s->bluetoothConnected = true;
      });
      return true;
    }
    delay(retryDelayMs);
  }
  return false;
}

// -------- Versão final: tenta NOME e (opcional) fallback por MAC --------
bool connectBluetooth()
{
  // 1) tenta pelo NOME (recomendado)
  if (connectBluetoothByName(targetBtName, 300, 15000)) {
    return true;
  }

  // 2) fallback por MAC (opcional; remova se não quiser)
  if (SerialBT.connect(elm327MacAddress)) {
    appStatus.mutualUpdate([](AppStatus* s){ s->bluetoothConnected = true; });
    return true;
  }

  return false;
}

bool connectElm327()
{
  bool elm327Connected = false;
  while (!elm327Connected)
  {
    // assinatura do begin permanece compatível no 3.x
    if (elm327.begin(SerialBT, false, 5000))
    {
      elm327Connected = true;
      appStatus.mutualUpdate(
          [](AppStatus *s)
          {
            s->elm327Connected = true;
          });
    }
    else
    {
      delay(200);
    }
  }

  return elm327Connected;
}

void elm327Task(void *arg)
{
  float accelerationMax = 1.0;  // m/s^2
  float decelerationMax = -1.0; // m/s^2

  while (1)
  {
    connectBluetooth();
    connectElm327();
    
    unsigned long lastFrequentTime = 0;
    unsigned long last1SecTime = 0;
    unsigned long lastKphTime = 0;
    float lastKph = 0.0;

    while (elm327.connected && SerialBT.connected())
    {
      // 3.x: nb_rx_state no lugar de status
      int8_t lastStatus = elm327.nb_rx_state;
      appStatus.mutualUpdate(
            [lastStatus](AppStatus *s)
            {
              s->lastElm327Status = lastStatus;
            });

      delay(50);

      // FIX: comparações de tempo estavam invertidas
      if ((millis() - lastFrequentTime) > 50)
      {
        lastFrequentTime = millis();

        // As funções retornam o valor e o estado vai para nb_rx_state
        float engineRpm = elm327.rpm();
        if (elm327.nb_rx_state == ELM_SUCCESS)
        {
          appStatus.mutualUpdate(
              [engineRpm](AppStatus *s)
              {
                s->engineRpm = engineRpm;
              });

          portYIELD();
        }
        else
        {
          continue;
        }

        float engineLoad = elm327.engineLoad();
        if (elm327.nb_rx_state == ELM_SUCCESS)
        {
          appStatus.mutualUpdate(
              [engineLoad](AppStatus *s)
              {
                s->engineLoad = engineLoad;
              });

          portYIELD();
        }
        else
        {
          continue;
        }
      }

      if ((millis() - lastKphTime) > 500)
      {
        unsigned long thisLastKphTime = lastKphTime;
        lastKphTime = millis();

        float kph = elm327.kph();
        if (elm327.nb_rx_state == ELM_SUCCESS)
        {
          // deltaSpeed em m/s
          float deltaSpeed = (kph - lastKph) * 1000.0f / 3600.0f;
          float deltaTime  = (float)((millis() - thisLastKphTime) / 1000.0f);
          float currentAccel = (deltaTime > 0.0f) ? (deltaSpeed / deltaTime) : 0.0f;

          if (currentAccel > 0 && currentAccel > accelerationMax) accelerationMax = currentAccel;
          if (currentAccel < 0 && currentAccel < decelerationMax) decelerationMax = currentAccel;

          float sign = (currentAccel >= 0.0f) ? 1.0f : -1.0f;
          float divider = (currentAccel >= 0.0f) ? accelerationMax : decelerationMax;
          // evita divisão por zero
          float accelPercent = (divider != 0.0f) ? (currentAccel / divider * sign * 100.0f) : 0.0f;

          lastKph = kph;

          appStatus.mutualUpdate(
              [accelPercent](AppStatus *s)
              {
                s->acceleration = accelPercent;
              });

          portYIELD();
        }
        else
        {
          lastKph = 0.0f;
          continue;
        }
      }
      
      if ((millis() - last1SecTime) >= 1000)
      {
        last1SecTime = millis();

        float engineCoolantTemp = elm327.engineCoolantTemp();
        if (elm327.nb_rx_state == ELM_SUCCESS)
        {
          appStatus.mutualUpdate(
              [engineCoolantTemp](AppStatus *s)
              {
                s->engineCoolantTemp = engineCoolantTemp;
              });

          portYIELD();
        }
        else
        {
          continue;
        }

        float ctrlModVoltage = elm327.ctrlModVoltage();
        if (elm327.nb_rx_state == ELM_SUCCESS)
        {
          appStatus.mutualUpdate(
              [ctrlModVoltage](AppStatus *s)
              {
                s->carBatVoltage = ctrlModVoltage;
              });

          portYIELD();
        }
        else
        {
          continue;
        }
      }
    }
  }
}

void powerTaskResetButton(void *arg)
{
  while (1)
  {
    if (M5_Safe.GetBtnPress() == 0x02)
    {
      M5.Beep.tone(1500);
      delay(100);
      M5.Beep.mute();
      ESP.restart();
    }

    delay(100);
  }
}

void powerTaskInfo(void *arg)
{
  while (1)
  {
    appStatus.mutualUpdate(
        [](
            AppStatus *s)
        {
          s->batVoltage = M5_Safe.GetBatVoltage();
        });

    delay(2000);
  }
}

void autoPowerOffTask(void *arg)
{
  bool extPowerInitial = M5_Safe.IsExternalPowerConnected();

  appStatus.mutualUpdate(
      [extPowerInitial](
          AppStatus *s)
      {
        s->extPower = extPowerInitial;
        s->powerOffScheduled = false;
      });

  while (1)
  {
    bool extPower = M5_Safe.IsExternalPowerConnected();

    appStatus.mutualUpdate(
        [extPower](
            AppStatus *s)
        {
          // Shutdown.
          if (!s->extPower && s->powerOffScheduled)
          {
            M5.Beep.tone(2000);
            delay(200);
            M5.Beep.mute();
            M5_Safe.PowerOff();
          }
          
          // Schedule shutdown, ext power unplugged.
          if (!extPower && s->extPower) 
          {
            s->powerOffScheduled = true;
            s->extPower = false;
            digitalWrite(10, LOW);
            return;
          } 

          // Battery low, schedule shutdown.
          if (!s->extPower && s->batVoltage < 3.6) 
          {
            s->powerOffScheduled = true;
            digitalWrite(10, LOW);
            return;
          } 

          // External power restored.
          if (!s->extPower && extPower)
          {
            s->powerOffScheduled = false;
            s->extPower = true;
            digitalWrite(10, HIGH);
            return;
          }
        });

    delay(5000);
  }
}

void screenRotationTask(void *arg)
{
  while (1)
  {
    float accX = M5_Safe.GetAccX();

    if (abs(accX) > 0.7)
    {
      appStatus.lock();

      if (accX < 0)
      {
        if (appStatus.screenRotation != 3)
        {          
          appStatus.screenRotation = 3;
          appStatus.screenRotationUpdated = true;
        }
      }
      else if (accX > 0)
      {
        if (appStatus.screenRotation != 1)
        {
          appStatus.screenRotation = 1;
          appStatus.screenRotationUpdated = true;
        }
      }

      appStatus.unlock();
    }

    delay(500);
  }
}

void setup()
{
  chipid = ESP.getEfuseMac();
  sprintf(chipname, "M5StickC_%04X", (uint16_t)(chipid >> 32));

  pinMode(10, OUTPUT);
  digitalWrite(10, HIGH);

  M5.begin();
  M5.Imu.Init();
  
  EEPROM.begin(sizeof(AppSettingsData));
  appSettings.load();

  // Inicia Bluetooth como MASTER e aplica PIN
  SerialBT.begin(chipname, true); // true = master
  SerialBT.setPin(targetBtPin);   // usa o PIN desejado

  xTaskCreatePinnedToCore(powerTaskInfo, "powerTaskInfo", configMINIMAL_STACK_SIZE + 1024, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(powerTaskResetButton, "powerTaskResetButton", configMINIMAL_STACK_SIZE + 1024, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(autoPowerOffTask, "autoPowerOffTask", configMINIMAL_STACK_SIZE + 1024, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(screenRotationTask, "screenRotationTask", configMINIMAL_STACK_SIZE + 1024, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(heartBeatTask, "heartBeatTask", configMINIMAL_STACK_SIZE + 1024, NULL, configMAX_PRIORITIES - 1, NULL, 0);
  xTaskCreatePinnedToCore(elm327Task, "elm327Task", configMINIMAL_STACK_SIZE + 1024 * 10, NULL, 0, NULL, 1);
}

void loop()
{
  appUI.update();
  delay(50);
}
