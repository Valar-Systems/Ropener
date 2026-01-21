volatile bool     g_buttonPressed = false;
volatile uint32_t g_buttonPressTime = -1;

void button_action(void)
{
  preferences.putInt ("wifi_set", 0);
  preferences.putString ("ssid", "NO_SSID");
  preferences.putString ("pass", "NO_PASSWORD");
  preferences.end();
  ESP.restart();
}

void button_change(void)
{
  Serial.println("WiFi Reset Pressed");
  bool buttonState = !digitalRead(WIFI_RESET_PIN);

  if (buttonState && !g_buttonPressed) {
    g_buttonPressTime = millis();
    g_buttonPressed = true;
    Serial.println("Hold the button to reset configuration...");
  } else if (!buttonState && g_buttonPressed) {
    g_buttonPressed = false;
    uint32_t buttonHoldTime = millis() - g_buttonPressTime;
    if (buttonHoldTime >= 5000) { // hold for 5 seconds to reset
      button_action();
    }
    g_buttonPressTime = -1;
  }
}