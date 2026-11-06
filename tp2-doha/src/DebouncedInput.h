#pragma once
#include <Arduino.h>
#include <FunctionalInterrupt.h>

class DebouncedInput
{
public:
    DebouncedInput(int pin, bool pullup = false, int debouncingTimerMs = 10)
        : _pin(pin), _debouncingDelayMs(debouncingTimerMs)
    {
        pinMode(_pin, pullup ? INPUT_PULLUP : INPUT);

        _stableState = digitalRead(_pin);
        _lastChange = millis();

        attachInterrupt(digitalPinToInterrupt(_pin), [this]() IRAM_ATTR
                        { this->tick(); }, CHANGE);
    }

    ~DebouncedInput()
    {
        detachInterrupt(digitalPinToInterrupt(_pin));
    }

    // Retourne la valeur debouncé du GPIO.
    bool read()
    {
        if (millis() - _lastChange >= _debouncingDelayMs)
        {
            _stableState = digitalRead(_pin);
        }
        return _stableState;
    }

    // Retourne la valeur sans debouncing du GPIO.
    bool readRaw()
    {
        return digitalRead(_pin);
    }

    // Appelé par l'ISR pour mettre à jour l'état de la broche.
    void IRAM_ATTR tick()
    {
        _lastChange = millis();
    }

    // Fonctions supprimées
    DebouncedInput(const DebouncedInput &) = delete;
    DebouncedInput(DebouncedInput &&) = delete;
    DebouncedInput &operator=(const DebouncedInput &) = delete;
    DebouncedInput &operator=(DebouncedInput &&) = delete;

private:
    int _pin;
    unsigned long _debouncingDelayMs;
    volatile unsigned long _lastChange;
    bool _stableState;
};